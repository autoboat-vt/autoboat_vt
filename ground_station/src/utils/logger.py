"""
Application logging for the ground station.

This module configures the standard library ``logging`` package for the entire
ground station.  All ``print()`` calls throughout the codebase should be
replaced with calls to the logger returned by :func:`get_logger`.

Two handlers are attached to the root logger:

- :class:`QtConsoleHandler` - emits records via a Qt signal so the in-app
  :class:`ConsoleOutputWidget` can display them with timestamps and syntax
  highlighting.  The formatted message keeps the ``[Info]``/``[Warning]``/
  ``[Error]`` prefix that :class:`ConsoleHighlighter` matches on.
- :class:`logging.handlers.RotatingFileHandler` - writes the same records to
  ``app_data/git_ignore/logs/ground_station.log`` (rotated at 5 MB, 5 backups)
  so a persistent record survives across runs.

Notes
-----
The Qt signal machinery is only wired up when
:func:`attach_console_widget` is called from ``ConsoleOutputWidget.__init__`` -
this avoids touching :class:`QObject` before :class:`QApplication` exists.  Before that
call, records still flow to the file handler and (if ``sys.stdout`` is a real
terminal) to the stream handler.
"""

from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path

from qtpy.QtCore import QObject, Signal

__all__ = [
    "ConsoleLogSignal",
    "QtConsoleHandler",
    "attach_console_widget",
    "get_logger",
    "root_logger",
]

_LEVEL_PREFIX: dict[int, str] = {
    logging.DEBUG: "[Debug]",
    logging.INFO: "[Info]",
    logging.WARNING: "[Warning]",
    logging.ERROR: "[Error]",
    logging.CRITICAL: "[Critical]",
}


class _PrefixFormatter(logging.Formatter):
    """
    Formatter that emits ``[Level] message`` (no timestamp).

    The console widget prepends its own timestamp, and :class:`ConsoleHighlighter`
    keys off the ``[Info]``/``[Warning]``/``[Error]`` prefix, so the formatter
    intentionally produces only the prefix + message.
    """

    def format(self, record: logging.LogRecord) -> str:
        """Format ``record`` as ``[Level] message``.

        Parameters
        ----------
        record
            The log record to format.

        Returns
        -------
        str
            The formatted log line.
        """

        prefix = _LEVEL_PREFIX.get(record.levelno, f"[{record.levelname}]")
        message = record.getMessage()
        if record.exc_info:
            message = f"{message}\n{self.formatException(record.exc_info)}"
        return f"{prefix} {message}"


class _FileFormatter(logging.Formatter):
    """
    Formatter for the on-disk log file.

    Includes timestamp, level, logger name, and message so the file is useful
    for post-mortem debugging without the console widget.
    """

    _FORMAT = "%(asctime)s %(levelname)-8s %(name)s: %(message)s"

    def __init__(self) -> None:
        super().__init__(fmt=self._FORMAT, datefmt="%Y-%m-%d %H:%M:%S")


class ConsoleLogSignal(QObject):
    """
    Qt object exposing the ``log_emitted`` signal.

    :class:`ConsoleOutputWidget` connects ``log_emitted`` to its append slot.  Kept
    as a separate :class:`QObject` so :class:`QtConsoleHandler` (which is a plain
    :class:`logging.Handler`) can emit across thread boundaries safely.
    """

    log_emitted = Signal(str)


class QtConsoleHandler(logging.Handler):
    """
    :class:`logging.Handler` that forwards records to the Qt console widget.

    The handler owns a :class:`ConsoleLogSignal` instance.  When
    :func:`attach_console_widget` is called, the console widget connects to
    ``signal.log_emitted`` and starts receiving formatted log lines.

    Notes
    -----
    Until :func:`attach_console_widget` is called, records are formatted and
    emitted to the signal but nothing is connected - this is fine because the
    console widget is created very early in ``MainWindow.__init__``.
    """

    def __init__(self) -> None:
        super().__init__()
        self._formatter = _PrefixFormatter()
        self.signal = ConsoleLogSignal()

    def emit(self, record: logging.LogRecord) -> None:
        """
        Format ``record`` and emit it on ``signal.log_emitted``.

        Parameters
        ----------
        record
            The log record to emit.
        """

        try:
            message = self._formatter.format(record)
        except Exception:
            self.handleError(record)
            return
        # Signal.emit is safe to call from any thread; Qt queues cross-thread
        # connections automatically.  Noop if nothing is connected.
        self.signal.log_emitted.emit(message)


# Singleton handler instances + root logger, created at import time so the first
# ``get_logger`` call from anywhere is already wired up.
_console_handler = QtConsoleHandler()
_console_handler.setLevel(logging.INFO)

_root_logger = logging.getLogger("ground_station")
_root_logger.setLevel(logging.DEBUG)  # handlers filter; root lets everything through
_root_logger.addHandler(_console_handler)
_root_logger.propagate = False  # avoid double-emitting to the root logger

_file_handler: RotatingFileHandler | None = None
_stream_handler: logging.StreamHandler | None = None


def _attach_file_handler(log_dir: Path) -> None:
    """
    Attach the rotating file handler to the root logger.

    Parameters
    ----------
    log_dir
        Directory to write the log file in.  Created if it doesn't exist.
    """

    global _file_handler  # noqa: PLW0603 - singleton handler, intentional

    if _file_handler is not None:
        return

    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "ground_station.log"
    handler = RotatingFileHandler(
        filename=log_path,
        maxBytes=5 * 1024 * 1024,  # 5 MB
        backupCount=5,
        encoding="utf-8",
    )
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(_FileFormatter())
    _root_logger.addHandler(handler)
    _file_handler = handler


def attach_console_widget(slot: object) -> None:
    """
    Connect the console widget's append slot to the Qt handler signal.

    Called from ``ConsoleOutputWidget.__init__`` once the widget exists.  Also
    attaches the file handler now that ``constants`` is fully imported (the
    logger module is imported very early, before ``constants.LOGS_DIR`` is
    defined, so file-handler setup is deferred to here).

    Parameters
    ----------
    slot
        Any callable accepted by ``Signal.connect`` - typically
        ``ConsoleOutputWidget.append_text``.
    """
    # Deferred import to avoid a circular dependency at module load time:
    # ``constants`` imports from ``utils`` (this package), and this function
    # needs ``constants.LOGS_DIR`` which is only defined inside the ``try``
    # block at the bottom of ``constants.py``.
    from utils import constants  # noqa: PLC0415 - deferred to break import cycle

    _attach_file_handler(constants.LOGS_DIR)
    _console_handler.signal.log_emitted.connect(slot)
    # Once the Qt console is attached, records flow to it via the Qt handler
    # AND to stdout via the stream handler.  The console widget captures
    # stdout too (via EmittingStream), which would double-display every line.
    # Drop the stream handler now that the Qt handler is live.
    global _stream_handler  # noqa: PLW0603 - singleton handler, intentional
    if _stream_handler is not None:
        _root_logger.removeHandler(_stream_handler)
        _stream_handler = None


def get_logger(name: str | None = None) -> logging.Logger:
    """
    Return a logger configured to publish to the console + file handlers.

    Parameters
    ----------
    name
        Logger name.  If ``None`` or ``"ground_station"``, the root ground
        station logger is returned.  Otherwise a child logger (e.g.
        ``"ground_station.widgets.groundstation"``) is returned - children
        inherit the handlers attached to the root.

    Returns
    -------
    logging.Logger
        The configured logger instance.

    Notes
    -----
    Callers should typically pass ``__name__`` so the logger name mirrors the
    module path, making it easy to filter output by subsystem.
    """

    if not name or name == "root":
        return _root_logger
    if name == "ground_station" or name.startswith("ground_station."):
        return logging.getLogger(name)
    return _root_logger.getChild(name)


# Module-level alias for callers that want the root logger directly.
root_logger: logging.Logger = _root_logger


# When running under ``run.sh`` (no Qt console yet), also mirror records to
# stdout so the launching terminal shows something.  Once the console widget
# attaches, ``attach_console_widget`` removes this handler to avoid
# double-display (the Qt handler takes over).
if sys.stdout is not None and not getattr(sys.stdout, "_is_autoboat_stream", False):
    _stream_handler = logging.StreamHandler(stream=sys.stdout)
    _stream_handler.setLevel(logging.INFO)
    _stream_handler.setFormatter(_PrefixFormatter())
    _root_logger.addHandler(_stream_handler)
