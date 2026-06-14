from __future__ import annotations

import contextlib
import fcntl
import json
from collections.abc import Callable, Generator
from functools import wraps
from os import fsync
from typing import Any, TextIO, TypeVar, cast

from utils import constants

T = TypeVar("T")

__all__ = ["StateManager"]

@contextlib.contextmanager
def _locked_file(path: constants.FileType, mode: str, lock_type: int) -> Generator[TextIO, None, None]:
    """
    Open a file and hold an advisory lock for the duration of the context.

    Parameters
    ----------
    path
        File path to open.
    mode
        File open mode.
    lock_type
        fcntl lock type, e.g. ``fcntl.LOCK_SH`` or ``fcntl.LOCK_EX``.

    Yields
    ------
    TextIO
        An open file object with the specified lock held.
    """

    with open(file=path, mode=mode, encoding="utf-8") as f:
        f = cast("TextIO", f)

        fcntl.flock(f, lock_type)
        try:
            yield f
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def _enforce_exact_return_type(expected_type: type[T]) -> Callable[[Callable[..., Any]], Callable[..., T | None]]:
    """
    Decorator to enforce that a function returns a value of the expected type or ``None``.

    Parameters
    ----------
    expected_type
        The type that the decorated function's return value must match exactly (not allowing subclasses).

    Returns
    -------
    Callable[[Callable[..., Any]], Callable[..., T | None]]
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
    dict[str, Any]
        The state data loaded from the file.

    Raises
    ------
    TypeError
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
    """Manage shared application state stored in a JSON file."""

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

        with _locked_file(path=constants.APP_STATE_PATH, mode="r+", lock_type=fcntl.LOCK_EX) as f:
            data = _load_state(f)
            data[variable] = value

            f.seek(0)
            json.dump(data, f, indent=4)
            f.truncate()
            f.flush()
            fsync(f.fileno())

    @staticmethod
    def _read(variable: str) -> Any | None:
        """
        Read a variable's value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to read.

        Returns
        -------
        Any | None
            The value associated with the variable, or ``None`` if the variable is not found.
        """

        with _locked_file(path=constants.APP_STATE_PATH, mode="r", lock_type=fcntl.LOCK_SH) as f:
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
        bool | None
            The value if it exists and is a boolean, otherwise ``None``.
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
        str | None
            The value if it exists and is a string, otherwise ``None``.
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
        int | None
            The value if it exists and is an integer, otherwise ``None``.
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
        float | None
            The value if it exists and is a float, otherwise ``None``.
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
        dict | None
            The value if it exists and is a dictionary, otherwise ``None``.
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
        list | None
            The value if it exists and is a list, otherwise ``None``.
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
        Any | None
            The value if it exists, otherwise ``None``.
        """

        return StateManager._read(variable)
