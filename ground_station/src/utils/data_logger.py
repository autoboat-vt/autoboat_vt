"""Module for managing the logging of data within the ground station."""

from __future__ import annotations

__all__ = ["DataLogger"]

import contextlib
import csv
import fcntl
from collections.abc import Generator
from datetime import datetime, timezone
from os import PathLike, fsync
from pathlib import Path
from typing import Any, TextIO


@contextlib.contextmanager
def _locked_file(path: str | PathLike[str], mode: str, lock_type: int) -> Generator[TextIO, None, None]:
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

    with open(path, mode=mode, encoding="utf-8", newline="") as f:
        fcntl.flock(f, lock_type)
        try:
            yield f
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


class DataLogger:
    """
    Class for managing the logging of data within the ground station.
    
    Parameters
    ----------
    file_path
        The file path to the data log file. If the file does not exist, it will be created.
    """

    __slots__ = ("_file_path",)
    _FIELDNAMES = ("write_time", "key_name", "data")

    def __init__(self, file_path: str | PathLike[str]) -> None:
        self._file_path = Path(file_path)
        
        if not self._file_path.exists():
            with _locked_file(self._file_path, "w", fcntl.LOCK_EX) as f:
                writer = csv.DictWriter(f, fieldnames=self._FIELDNAMES, lineterminator="\n")
                writer.writeheader()
                f.flush()
                fsync(f.fileno())

    @property
    def file_path(self) -> Path:
        """
        Get the file path of the data logger.

        Returns
        -------
        Path
            The file path of the data logger.
        """

        return self._file_path

    def write(self, key_name: str, data: Any) -> None:
        """
        Append a new log entry to the CSV file.

        Parameters
        ----------
        key_name
            The key or field name being logged.
        data
            The value associated with the key.
        """

        row = {
            "write_time": datetime.now(timezone).isoformat(),
            "key_name": key_name,
            "data": str(data),
        }

        with _locked_file(self._file_path, "a", fcntl.LOCK_EX) as f:
            writer = csv.DictWriter(f, fieldnames=self._FIELDNAMES, lineterminator="\n")
            writer.writerow(row)
            f.flush()
            fsync(f.fileno())
