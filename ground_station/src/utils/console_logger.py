from __future__ import annotations

import logging
from contextlib import suppress
from datetime import datetime, timezone
from functools import cache
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import TYPE_CHECKING

from qtpy.QtCore import QObject, Signal

if TYPE_CHECKING:
    from collections.abc import Callable

__all__ = [
    "attach_console_widget",
    "get_logger",
    "root_logger",
]

_PACKAGE_NAME = "ground_station"

_LEVEL_PREFIX: dict[int, str] = {
    logging.DEBUG: "[Debug]",
    logging.INFO: "[Info]",
    logging.WARNING: "[Warning]",
    logging.ERROR: "[Error]",
    logging.CRITICAL: "[Critical]",
}


class _PrefixFormatter(logging.Formatter):
    """Formatter that emits ``[Level] message`` (no timestamp)."""

    def format(self, record: logging.LogRecord) -> str:
        """
        Format ``record`` as ``[Level] message``.

        Parameters
        ----------
        record
            The log record to format.

        Returns
        -------
        `str`
            The formatted log line.
        """

        prefix = _LEVEL_PREFIX.get(record.levelno, f"[{record.levelname}]")
        message = record.getMessage()
        if record.exc_info:
            message = f"{message}\n{self.formatException(record.exc_info)}"

        return f"{prefix} {message}"


class _ConsoleLogSignal(QObject):
    """Carry log messages across Qt threads."""

    log_emitted = Signal(str)


class _QtConsoleHandler(logging.Handler):
    """
    Forward log records to the Qt console widget through a signal.

    Records are dropped until a slot is attached via :meth:`attach`. This allows
    the handler to be created at import time (so early startup logs are captured)
    without requiring the console widget to exist yet. Once the widget is ready,
    :func:`attach_console_widget` is called to connect the widget's slot and switch
    the handler to live mode.
    """

    def __init__(self) -> None:
        super().__init__()

        self.signal = _ConsoleLogSignal()
        self._formatter = _PrefixFormatter()
        self._attached = False

    def emit(self, record: logging.LogRecord) -> None:
        """
        Forward an attached log record to the Qt console signal.

        Parameters
        ----------
        record
            Logging record to format or emit.
        """

        if not self._attached:
            return

        try:
            self.signal.log_emitted.emit(self._formatter.format(record))
        except Exception:
            self.handleError(record)

    def attach(self, slot: Callable[[str], None]) -> None:
        """
        Connect a console slot and enable Qt log delivery.

        Parameters
        ----------
        slot
            Qt-compatible callable that receives formatted log messages.
        """

        self.signal.log_emitted.connect(slot)
        self._attached = True


def _default_log_dir() -> Path:
    """
    Locate ``app_data/git_ignore/logs`` without importing ``constants``.

    Mirrors ``constants.LOGS_DIR`` (``cwd / "app_data" / "git_ignore" / "logs"``)
    so the file handler at import time writes to the same place it would have
    if attached later via :func:`attach_console_widget`.

    Returns
    -------
    :class:`Path`
        The default log directory path.
    """

    return Path.cwd() / "app_data" / "git_ignore" / "console_logs"


class _HandlerRegistry:
    """
    Manage the file handler attached to the root logger.

    Parameters
    ----------
    logger
        The root logger to which the file handler will be attached.
    """

    def __init__(self, logger: logging.Logger) -> None:
        self._logger = logger
        self._file: RotatingFileHandler | None = None

    def install_file(self, log_dir: Path) -> None:
        """
        Install a rotating file handler on the root logger.

        Parameters
        ----------
        log_dir
            Directory where the log file will be created. The directory is
            created if it does not exist.
        """

        if self._file is not None:
            return

        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        handler = RotatingFileHandler(
            filename=log_dir / f"{timestamp}.log",
            encoding="utf-8",
        )
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(
            logging.Formatter(
                fmt="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ),
        )
        self._logger.addHandler(handler)
        self._file = handler


_qt_handler = _QtConsoleHandler()
_qt_handler.setLevel(logging.INFO)

_root_logger = logging.getLogger(_PACKAGE_NAME)
_root_logger.setLevel(logging.DEBUG)
_root_logger.addHandler(_qt_handler)
_root_logger.propagate = False

_handlers = _HandlerRegistry(_root_logger)

# attach the file handler at import time so early startup logs are captured, but
# suppress errors in read-only environments (e.g. Qt console)
with suppress(OSError):
    _handlers.install_file(_default_log_dir())


@cache
def _cached_logger(name: str) -> logging.Logger:
    """
    Return a cached child of the root logger.

    Parameters
    ----------
    name
        Name of the logger to retrieve. It must be a child of the root logger.

    Returns
    -------
    :class:`logging.Logger`
        A logger whose records flow to the Qt console and file handlers.
    """

    return _root_logger.getChild(name)


def get_logger(name: str | None = None) -> logging.Logger:
    """
    Return a logger configured to publish to the console + file handlers.

    Callers should typically pass ``__name__`` so the logger name mirrors the
    module path, making it easy to filter output by subsystem.

    Parameters
    ----------
    name
        Logger name.  If ``None`` or ``"ground_station"``, the root ground
        station logger is returned.  Otherwise a child logger (e.g.
        ``"ground_station.widgets.groundstation"``) is returned - children
        inherit the handlers attached to the root.

    Returns
    -------
    :class:`logging.Logger`
        The configured logger instance.
    """

    if not name or name == "root":
        return _root_logger

    if name == _PACKAGE_NAME or name.startswith(f"{_PACKAGE_NAME}."):
        return logging.getLogger(name)

    return _cached_logger(name)


def attach_console_widget(slot: Callable[[str], None]) -> None:
    """
    Connect the console widget's append slot to the Qt handler signal.

    Called from :meth:`ConsoleOutputWidget.__init__` once the widget exists.  Also
    attaches the file handler now that ``constants`` is fully imported (the
    logger module is imported very early, before ``constants.LOGS_DIR`` is
    defined, so file-handler setup is deferred to here).

    Parameters
    ----------
    slot
        Any callable accepted by :meth:`Signal.connect`.
    """

    _qt_handler.attach(slot)


# expose the root logger for convenience, but discourage its use in favor of
# get_logger(__name__) so the logger name mirrors the module path
root_logger: logging.Logger = _root_logger
