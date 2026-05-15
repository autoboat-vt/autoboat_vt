"""Module for managing the logging of data within the ground station."""

from __future__ import annotations

__all__ = ["DataLogEntry", "DataLogger"]

import contextlib
import fcntl
from collections.abc import Generator, Iterable
from csv import DictWriter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from os import fsync
from pathlib import Path
from typing import Any, TextIO, cast

from qtpy.QtCore import Slot

from utils import constants


@dataclass
class DataLogEntry:
    """Data class representing a single log entry."""

    key_name: str
    data: object
    write_time: datetime = field(default_factory=lambda: datetime.now(timezone.utc))

    def as_dict(self) -> dict[str, str]:
        """
        Convert the log entry to a dictionary.

        Returns
        -------
        dict[str, str]
            A dictionary representation of the log entry.
        """

        return {
            "key_name": self.key_name,
            "data": str(self.data),
            "write_time": self.write_time.isoformat(),
        }


_FIELDNAMES = list(DataLogEntry.__annotations__.keys())


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

    with open(path, mode=mode, encoding="utf-8", newline="") as f:
        f = cast("TextIO", f)

        fcntl.flock(f, lock_type)
        try:
            yield f
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def _load_log() -> Path:
    """
    Load the CSV log file, creating it with a header if it doesn't exist.

    Returns
    -------
    Path
        The path to the log file.

    Raises
    ------
    RuntimeError
        If the log file cannot be loaded or created.
    """

    log = Path(constants.SM.read("data_log_file_path"))

    if not log.exists():
        print(f"[Info] Creating new data log file at {log}...")

    try:
        with _locked_file(path=log, mode="a+", lock_type=fcntl.LOCK_EX) as f:
            f.seek(0, 2)

            if f.tell() == 0:
                writer = DictWriter(f, fieldnames=_FIELDNAMES, lineterminator="\n")
                writer.writeheader()
                f.flush()
                fsync(f.fileno())

    except Exception as e:
        raise RuntimeError(f"Failed to load data log file {log}: {e}") from e

    return log


class DataLogger:
    """Class for managing the logging of data within the ground station."""

    @staticmethod
    def write(key_name: str, data: object) -> None:
        """
        Append a new log entry to the CSV file.

        Parameters
        ----------
        key_name
            The key or field name being logged.
        data
            The value associated with the key.
        """

        DataLogger.bulk_write([DataLogEntry(key_name=key_name, data=data)])

    @staticmethod
    def bulk_write(entries: Iterable[tuple[str, object] | DataLogEntry]) -> None:
        """
        Append multiple log entries to the CSV file in a single operation.

        Parameters
        ----------
        entries
            Tuples containing key names and data values, or ``DataLogEntry`` objects.
        """

        rows: list[dict[str, str]] = []
        for entry in entries:
            if isinstance(entry, DataLogEntry):
                log_entry = entry
            else:
                key_name, data = entry
                log_entry = DataLogEntry(key_name=key_name, data=data)

            if not log_entry.key_name.strip():
                print(f"[Warning] Skipping log entry with empty key name and data '{log_entry.data}'")
                continue

            rows.append(log_entry.as_dict())

        if not rows:
            return

        log_file = _load_log()

        with _locked_file(path=log_file, mode="a", lock_type=fcntl.LOCK_EX) as f:
            writer = DictWriter(f, fieldnames=_FIELDNAMES, lineterminator="\n")
            writer.writerows(rows)
            f.flush()
            fsync(f.fileno())

    @staticmethod
    @Slot(tuple)
    def write_from_qthread(request_result: tuple[dict[str, Any], constants.TelemetryStatus]) -> None:
        """
        Convenience method for writing log entries from a ``QThread``, where the data is returned as a tuple.

        Parameters
        ----------
        request_result
            A tuple containing:
            - a dictionary with the latest boat telemetry data.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        boat_data, _ = request_result

        DataLogger.bulk_write(DataLogEntry(key_name=key, data=value) for key, value in boat_data.items())
