"""Module for managing the state of variables within the ground station."""

from __future__ import annotations

__all__ = ["LockStatus", "StateManager"]

import contextlib
import fcntl
import json
import time
from collections.abc import Callable, Generator
from enum import IntEnum
from functools import wraps
from typing import Any, ParamSpec, TypeVar

from utils import constants

P = ParamSpec("P")
R = TypeVar("R")

class LockStatus(IntEnum):
    """
    Enum representing the lock status of the state file.

    Attributes
    ----------
    LOCKED
        The state file is locked, indicating that it is currently being accessed or modified.
    UNLOCKED
        The state file is unlocked, indicating that it is not currently being accessed or modified.

    Inherits
    -------
    ``IntEnum``
    """
    
    LOCKED = 1
    UNLOCKED = 0


def _check_lock(func: Callable[P, R]) -> Callable[P, R]:
    """
    Decorator to check the lock status before executing a function.

    Parameters
    ----------
    func
        The function to be decorated.

    Returns
    -------
    Callable[P, R]
        The decorated function.
    """

    @wraps(func)
    def wrapper(self: StateManager, *args: P.args, **kwargs: P.kwargs) -> R:
        while self._lock == LockStatus.LOCKED:
            time.sleep(0.01)

        return func(self, *args, **kwargs)

    return wrapper


@contextlib.contextmanager
def _locked_file(path: str, mode: str, lock_type: int) -> Generator[Any, None, None]:
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
    """

    with open(file=path, mode=mode, encoding="utf-8") as f:
        fcntl.flock(f, lock_type)
        try:
            yield f
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)

class StateManager:
    """Class responsible for managing the state of variables within the groundstation which are used in multiple places."""

    __slots__ = ("_lock",)

    def __init__(self) -> None:
        self._lock = LockStatus.UNLOCKED

    @property
    def lock(self) -> LockStatus:
        """
        Get the current lock state.
        
        Returns
        -------
        LockStatus
            The current lock state of the state file.
        """

        return self._lock
    
    @_check_lock
    def write(self, variable: str, value: Any) -> None:
        """
        Write a variable and its value to the state file.

        Parameters
        ----------
        variable
            The name of the variable to be written.
        value
            The value of the variable to be written.
        """

        self._lock = LockStatus.LOCKED
        try:
            with _locked_file(path=constants.APP_STATE_PATH, mode="r+", lock_type=fcntl.LOCK_EX) as f:
                data: dict[str, Any] = json.load(f)
                data[variable] = value

                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()
        
        finally:
            self._lock = LockStatus.UNLOCKED

    @_check_lock
    def read(self, variable: str) -> Any:
        """
        Read a variable's value from the state file.

        Parameters
        ----------
        variable
            The name of the variable to be read.

        Returns
        -------
        Any
            The value of the variable read from the state file.
        """

        self._lock = LockStatus.LOCKED
        try:
            with _locked_file(path=constants.APP_STATE_PATH, mode="r", lock_type=fcntl.LOCK_SH) as f:
                data: dict[str, Any] = json.load(f)
                return data.get(variable)

        finally:
            self._lock = LockStatus.UNLOCKED
