"""Module containing the :class:`QWebEnginePage` subclass used to host the map frontend."""

from __future__ import annotations

from collections.abc import Callable

from qtpy.QtWebEngineWidgets import QWebEnginePage

from utils.console_logger import get_logger

__all__ = ["MapWebEnginePage"]

logger = get_logger(__name__)

_LEVEL_LOGGERS: dict[QWebEnginePage.JavaScriptConsoleMessageLevel, Callable[[str], None]] = {
    QWebEnginePage.JavaScriptConsoleMessageLevel.InfoMessageLevel: logger.info,
    QWebEnginePage.JavaScriptConsoleMessageLevel.WarningMessageLevel: logger.warning,
    QWebEnginePage.JavaScriptConsoleMessageLevel.ErrorMessageLevel: logger.error,
}


class MapWebEnginePage(QWebEnginePage):
    """
    A :class:`QWebEnginePage` that forwards the frontend's JavaScript console output to the ground station logger.

    The map widget is rendered inside a :class:`QWebEngineView`, which has no visible developer console, so
    messages logged from the Typescript frontend (for example the MapTiler key fallback warning in `main.ts`)
    would otherwise never be seen. Every ``console.log``/``console.warn``/``console.error`` is re-emitted as a
    `[map]` log record, which reaches both the Qt console widget and the log file.

    Inherits
    --------
    :class:`QWebEnginePage`
    """

    def javaScriptConsoleMessage(
        self,
        level: QWebEnginePage.JavaScriptConsoleMessageLevel,
        message: str,
        line_number: int,
        source_id: str,
    ) -> None:
        """
        Forward a JavaScript console message to the Qt logger.

        Parameters
        ----------
        level
            Severity reported by Chromium; mapped to the matching logger method.
        message
            The text passed to the JavaScript ``console`` call.
        line_number
            Line number in the source file that logged the message.
        source_id
            URL of the source file that logged the message.
        """

        log = _LEVEL_LOGGERS.get(level, logger.info)
        source = source_id.rsplit("/", 1)[-1] if source_id else "map"
        log(f"[map] {message} ({source}:{line_number})")
