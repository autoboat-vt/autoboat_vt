from __future__ import annotations

from threading import Condition

from qtpy.QtCore import QObject, Signal

from utils.console_logger import get_logger

__all__ = ["LAND_CLICK_PROMPT", "LandClickPrompt"]

logger = get_logger(__name__)


class LandClickPrompt(QObject):
    """
    Thread-safe question/answer channel for land confirmation dialogs.

    The HTTP server thread calls :meth:`ask`, which emits
    :attr:`agreement_requested`. The slot that runs on the Qt main thread must
    show the dialog and then call :meth:`answer` with the user's choice. Only
    one prompt may be in flight at a time; concurrent land clicks are blocked
    until the first is resolved.

    Inherits
    --------
    :class:`QObject`
    """

    agreement_requested = Signal(float, float)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._condition = Condition()
        self._answer: bool | None = None
        self._pending = False

    def ask(self, latitude: float, longitude: float) -> bool:
        """
        Block the calling thread until the user answers the confirmation dialog.

        Must only be called from the HTTP server thread. Falls back to "do not
        add" if a prompt is already in flight (caller should proceed by treating
        the click as water instead).

        Parameters
        ----------
        latitude
            Latitude of the clicked point.
        longitude
            Longitude of the clicked point.

        Returns
        -------
        bool
            `True` if the user confirmed the waypoint should be added,
            `False` otherwise.
        """

        with self._condition:
            if self._pending:
                logger.warning("Land confirmation already in progress; defaulting to add. ")
                return True

            self._pending = True
            self._answer = None

            self.agreement_requested.emit(latitude, longitude)

            while self._answer is None:
                self._condition.wait(timeout=0.1)

            self._pending = False
            return self._answer

    def answer(self, add_waypoint: bool) -> None:
        """
        Post the user's answer back to the blocked HTTP thread.

        Parameters
        ----------
        add_waypoint
            `True` if the user chose to add the waypoint anyway.
        """

        with self._condition:
            if not self._pending:
                logger.debug("Received land confirmation answer with no pending prompt.")
                return

            self._answer = add_waypoint
            self._condition.notify_all()


# singleton used by both the HTTP handler and the GroundStationWidget
LAND_CLICK_PROMPT = LandClickPrompt()
