from __future__ import annotations

import contextlib
import json
import threading
from collections.abc import Callable, Generator
from functools import wraps
from os import fsync
from typing import Any, ClassVar, TextIO, TypeVar, cast

from utils import constants
from utils.file_lock import lock_exclusive, lock_shared, unlock

__all__ = ["StateManager"]

T = TypeVar("T")


@contextlib.contextmanager
def _locked_file(path: constants.FileType, mode: str, exclusive: bool) -> Generator[TextIO, None, None]:
    """
    Open a file and hold an advisory lock for the duration of the context.

    Parameters
    ----------
    path
        File path to open.
    mode
        File open mode.
    exclusive
        When `True`, take an exclusive (write) lock; otherwise a shared
        (read) lock. See :mod:`utils.file_lock` for platform behavior.

    Yields
    ------
    :class:`TextIO`
        An open file object with the specified lock held.
    """

    with open(file=path, mode=mode, encoding="utf-8") as f:
        f = cast("TextIO", f)

        lock_exclusive(f) if exclusive else lock_shared(f)
        try:
            yield f
        finally:
            unlock(f)


def _enforce_exact_return_type(expected_type: type[T]) -> Callable[[Callable[..., Any]], Callable[..., T | None]]:
    """
    Decorator to enforce that a function returns a value of the expected type or ``None``.

    Parameters
    ----------
    expected_type
        The type that the decorated function's return value must match exactly (not allowing subclasses).

    Returns
    -------
    `Callable[[Callable[..., Any]], Callable[..., T | None]]`
        A decorator that can be applied to functions to enforce the return type.
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., T | None]:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> T | None:
            value = func(*args, **kwargs)

            if value is None:
                return None

            if type(value) is expected_type:
                return cast("T", value)

            return None

        return wrapper

    return decorator


def _load_state(f: TextIO) -> dict[str, Any]:
    """
    Load the state from the given file object.

    Parameters
    ----------
    f
        An open file object to read the state from.

    Returns
    -------
    `dict[str, Any]`
        The state data loaded from the file.

    Raises
    ------
    :class:`TypeError`
        If the file does not contain a JSON object.
    """

    f.seek(0)
    content = f.read().strip()
    if not content:
        return {}

    data = json.loads(content)

    if not isinstance(data, dict):
        raise TypeError(f"State file must contain a JSON object. Found {type(data).__name__} instead.")

    return data


class StateManager:
    """
    Manage shared application state stored in a JSON file.

    Attributes
    ----------
    _cache
        Read-through cache of the state file contents.
    _cache_lock
        Lock guarding concurrent access to the cache.
    """

    _cache: ClassVar[dict[str, Any]] = {}
    _cache_lock: ClassVar[threading.Lock] = threading.Lock()

    @staticmethod
    def _invalidate_cache() -> None:
        """Clear the read-through cache. Called after every write."""

        with StateManager._cache_lock:
            StateManager._cache.clear()

    @staticmethod
    def write(variable: str, value: Any) -> None:
        """
        Write a variable and its value to the state file.

        Parameters
        ----------
        variable
            The name of the variable to write.
        value
            The value to associate with the variable.
        """

        constants.APP_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        if not constants.APP_STATE_PATH.exists():
            constants.APP_STATE_PATH.write_text(json.dumps(constants.STATE_FILE_CONTENTS, indent=4))

        with _locked_file(path=constants.APP_STATE_PATH, mode="r+", exclusive=True) as f:
            data = _load_state(f)
            data[variable] = value

            f.seek(0)
            json.dump(data, f, indent=4)
            f.truncate()
            f.flush()
            fsync(f.fileno())

        with StateManager._cache_lock:
            StateManager._cache.clear()
            StateManager._cache[variable] = value

    @staticmethod
    def _read(variable: str) -> Any | None:
        """
        Read a variable's value from the state file.

        Uses a process-wide read-through cache to avoid re-parsing the JSON
        file on every call. The cache is invalidated by :meth:`write`; this is
        safe because writes are rare relative to reads, and any external
        process modifying the file out-of-band is not a supported pattern
        in this application.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `Any | None`
            The value associated with the variable, or `None` if the variable is not found.
        """

        with StateManager._cache_lock:
            if variable in StateManager._cache:
                return StateManager._cache[variable]

        # The state file may legitimately not exist yet during very early
        # startup (before constants.py's bootstrap finishes) or transiently
        # while an exit-path cleanup removes it. Missing file == empty state.
        if not constants.APP_STATE_PATH.is_file():
            return None

        with _locked_file(path=constants.APP_STATE_PATH, mode="r", exclusive=False) as f:
            data = _load_state(f)
            return data.get(variable)

    @staticmethod
    @_enforce_exact_return_type(bool)
    def read_bool(variable: str) -> bool | None:
        """
        Read a boolean value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `bool | None`
            The value if it exists and is a boolean, otherwise `None`.
        """

        return StateManager._read(variable)

    @staticmethod
    @_enforce_exact_return_type(str)
    def read_str(variable: str) -> str | None:
        """
        Read a string value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `str | None`
            The value if it exists and is a string, otherwise `None`.
        """

        return StateManager._read(variable)

    @staticmethod
    @_enforce_exact_return_type(int)
    def read_int(variable: str) -> int | None:
        """
        Read an integer value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `int | None`
            The value if it exists and is an integer, otherwise `None`.
        """

        return StateManager._read(variable)

    @staticmethod
    @_enforce_exact_return_type(float)
    def read_float(variable: str) -> float | None:
        """
        Read a float value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `float | None`
            The value if it exists and is a float, otherwise `None`.
        """

        return StateManager._read(variable)

    @staticmethod
    @_enforce_exact_return_type(dict)
    def read_dict(variable: str) -> dict | None:
        """
        Read a dictionary value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `dict | None`
            The value if it exists and is a dictionary, otherwise `None`.
        """

        return StateManager._read(variable)

    @staticmethod
    @_enforce_exact_return_type(list)
    def read_list(variable: str) -> list | None:
        """
        Read a list value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `list | None`
            The value if it exists and is a list, otherwise `None`.
        """

        return StateManager._read(variable)

    @staticmethod
    @_enforce_exact_return_type(Any)
    def read_any(variable: str) -> Any | None:
        """
        Read a value of any type from the state file without enforcing a specific type.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        `Any | None`
            The value if it exists, otherwise `None`.
        """

        return StateManager._read(variable)
