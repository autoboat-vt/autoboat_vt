"""Module for managing the state of variables within the ground station."""

from __future__ import annotations

__all__ = ["StateManager"]

import contextlib
import fcntl
import json
from collections.abc import Generator
from os import fsync
from typing import Any, TextIO, cast

from utils import constants


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


class StateManager:
    """Manage shared application state stored in a JSON file."""

    __slots__ = ("_path",)

    def _load_state(self, f: TextIO) -> dict[str, Any]:
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
            raise TypeError("State file must contain a JSON object.")
        
        if not all(isinstance(key, str) for key in data):
            raise TypeError("State file JSON object must only contain string keys.")

        return data

    def write(self, variable: str, value: Any) -> None:
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
            data = self._load_state(f)
            data[variable] = value

            f.seek(0)
            json.dump(data, f, indent=4)
            f.truncate()
            f.flush()
            fsync(f.fileno())

    def read(self, variable: str) -> Any | None:
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
            data = self._load_state(f)
            return data.get(variable)
