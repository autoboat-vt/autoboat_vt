"""Module for managing the logging of data within the ground station."""

from __future__ import annotations

__all__ = ["DataLogger"]

import contextlib
import csv
import fcntl
from collections.abc import Generator
from datetime import datetime, timezone
from os import fsync
from pathlib import Path
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

    with open(path, mode=mode, encoding="utf-8", newline="") as f:
        f = cast("TextIO", f)

        fcntl.flock(f, lock_type)
        try:
            yield f
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


class DataLogger:
    """Class for managing the logging of data within the ground station."""

    _FIELDNAMES: tuple[str, str, str] = ("write_time", "key_name", "data")

    def __init__(self) -> None:
        self._file_path = Path(constants.SM.read("data_log_file_path"))

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

    def write(self, key_name: str, data: object) -> None:
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
            "write_time": datetime.now(timezone.utc).isoformat(),
            "key_name": key_name,
            "data": str(data),
        }

        try:
            with _locked_file(self._file_path, "a", fcntl.LOCK_EX) as f:
                # if missing header
                if f.tell() == 0:
                    raise RuntimeError(f"Data log file {self._file_path} is missing header row.")

                writer = csv.DictWriter(f, fieldnames=self._FIELDNAMES, lineterminator="\n")
                writer.writerow(row)
                f.flush()
                fsync(f.fileno())

        except Exception as e:
            print(f"[Error] Failed to write to data log file {self._file_path}: {e}")

    def bulk_write(self, entries: list[tuple[str, object]]) -> None:
        """
        Append multiple log entries to the CSV file in a single operation.

        Parameters
        ----------
        entries
            A list of tuples, where each tuple contains a key name and its associated data.
        """

        write_time = datetime.now(timezone.utc).isoformat()

        rows = [
            {
                "write_time": write_time,
                "key_name": key_name,
                "data": str(data),
            }
            for key_name, data in entries
        ]

        try:
            with _locked_file(self._file_path, "a", fcntl.LOCK_EX) as f:
                # if missing header
                if f.tell() == 0:
                    raise RuntimeError(f"Data log file {self._file_path} is missing header row.")

                writer = csv.DictWriter(f, fieldnames=self._FIELDNAMES, lineterminator="\n")
                writer.writerows(rows)
                f.flush()
                fsync(f.fileno())

        except Exception as e:
            print(f"[Error] Failed to write to data log file {self._file_path}: {e}")

    def write_from_qthread(self, request_result: tuple[dict[str, Any], constants.TelemetryStatus]) -> None:
        """
        Write log entries from a ``QThread``, which may be running in a different thread context.

        Parameters
        ----------
        request_result
            A tuple containing:
                - a dictionary of boat status,
                - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        boat_status, telemetry_status = request_result

        if telemetry_status == constants.TelemetryStatus.SUCCESS:
            entries = [
                (key, value)
                for key, value in boat_status.items()
                if key in constants.SM.read("data_log_keys")
            ]
            self.bulk_write(entries)
