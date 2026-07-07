from __future__ import annotations

import contextlib
import fcntl
import threading
from collections.abc import Generator, Iterable
from csv import DictWriter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from os import fsync
from pathlib import Path
from typing import Any, TextIO, cast

from qtpy.QtCore import QObject, QTimer, Slot

from utils import constants

__all__ = ["DataLogger"]

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


def _load_log(header_written: bool = False) -> Path:
    """
    Load the CSV log file, creating it with a header if it doesn't exist.

    Parameters
    ----------
    header_written
        Whether the CSV header has already been written during this logging
        session. When ``True``, the file is known to exist and be initialized,
        so this function avoids re-opening it in ``a+`` mode just to check.

    Returns
    -------
    Path
        The path to the log file.

    Raises
    ------
    RuntimeError
        If the log file cannot be loaded or created.
    """

    log = Path(constants.SM.read_str("data_log_file_path"))

    if header_written:
        return log

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


class DataLogger(QObject):
    """Class for managing the logging of data within the ground station.

    Telemetry arrives at a high rate via ``write_from_qthread``. Writing each
    tick straight to disk would mean one file open, one ``flock``, and one
    ``fsync`` call per tick on the GUI thread. Instead, incoming entries are
    appended to an in-memory buffer and drained by a QTimer every
    ``_FLUSH_INTERVAL_MS`` milliseconds in a single batched write.
    """

    _FLUSH_INTERVAL_MS = 1_000

    def __init__(self) -> None:
        super().__init__()
        self._buffer: list[dict[str, str]] = []
        self._buffer_lock = threading.Lock()
        self._header_written = False

        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(self._FLUSH_INTERVAL_MS)
        self._flush_timer.timeout.connect(self._flush)

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

        self.bulk_write([DataLogEntry(key_name=key_name, data=data)])

    def bulk_write(self, entries: Iterable[tuple[str, object] | DataLogEntry]) -> None:
        """
        Append multiple log entries to the buffer for later flushing.

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

        with self._buffer_lock:
            self._buffer.extend(rows)

    @Slot(tuple)
    def write_from_qthread(self, request_result: tuple[dict[str, Any], constants.TelemetryStatus]) -> None:
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

        self.bulk_write(DataLogEntry(key_name=key, data=value) for key, value in boat_data.items())

    @Slot()
    def start(self) -> None:
        """Start the periodic flush timer. Called when logging begins."""

        self._header_written = False
        self._flush_timer.start()

    @Slot()
    def stop(self) -> None:
        """Stop the flush timer and drain any buffered entries. Called when logging ends."""

        self._flush_timer.stop()
        self._flush()

    @Slot()
    def _flush(self) -> None:
        """Drain the buffer and write all pending entries to disk in one batch."""

        with self._buffer_lock:
            if not self._buffer:
                return
            
            rows = self._buffer
            self._buffer = []

        log_file = _load_log(header_written=self._header_written)
        self._header_written = True

        with _locked_file(path=log_file, mode="a", lock_type=fcntl.LOCK_EX) as f:
            writer = DictWriter(f, fieldnames=_FIELDNAMES, lineterminator="\n")
            writer.writerows(rows)
            f.flush()
            fsync(f.fileno())
