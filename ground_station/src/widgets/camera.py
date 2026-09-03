from __future__ import annotations

from qtpy.QtCore import QByteArray, Qt
from qtpy.QtGui import QPixmap, QResizeEvent
from qtpy.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QWidget

from utils import constants, misc, thread_classes
from utils.console_logger import get_logger

__all__ = ["CameraWidget"]

logger = get_logger(__name__)


class CameraWidget(QWidget):
    """
    A widget to display a camera feed using :class:`QLabel` + :class:`QPixmap`.

    Inherits
    -------
    :class:`QWidget`
    """

    def __init__(self) -> None:
        super().__init__()
        self.main_layout = QGridLayout()

        self.controls_layout = QHBoxLayout()

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.pause_timer)
        self.is_paused = True
        self.paused_icon_base64 = open(constants.ASSETS_DIR / "new_logo-base64.txt", encoding="utf-8").read()
        self.pause_button.setDisabled(not self.is_paused)

        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self.unpause_timer)
        self.is_running = False

        self.controls_layout.addWidget(self.pause_button)
        self.controls_layout.addWidget(self.run_button)
        self.main_layout.addLayout(self.controls_layout, 1, 0)

        self.web_view_layout = QHBoxLayout()

        self.current_pixmap: QPixmap | None = None
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("background-color: black;")

        self.image_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.image_label.setMinimumSize(1, 1)

        self.web_view_layout.addWidget(self.image_label)
        self.main_layout.addLayout(self.web_view_layout, 0, 0)
        self.setLayout(self.main_layout)

        self.image_fetcher = thread_classes.ImageFetcher()
        self.image_fetcher.data_fetched.connect(self.update_camera_feed)

        self.timer = misc.copy_qtimer(constants.HALF_SECOND_TIMER)
        self.timer.timeout.connect(self.image_fetcher.get_image)

    def unpause_timer(self) -> None:
        """Unpause the timer that fetches images from the camera."""

        self.timer.start()
        self.is_running = True
        self.is_paused = False
        self.run_button.setDisabled(self.is_running)
        self.pause_button.setDisabled(self.is_paused)
        logger.info("Unpaused camera feed timer.")

    def pause_timer(self) -> None:
        """Pause the timer that fetches images from the camera."""

        self.timer.stop()
        self.is_running = False
        self.is_paused = True
        self.update_camera_feed(self.paused_icon_base64)
        self.pause_button.setDisabled(self.is_paused)
        self.run_button.setDisabled(self.is_running)
        logger.info("Paused camera feed timer.")

    def update_camera_feed_starter(self) -> None:
        """Start the image fetcher thread to update the camera feed if it is not already running."""

        if not self.image_fetcher.isRunning():
            self.image_fetcher.start()

    def update_camera_feed(self, base64_encoded_image: str) -> None:
        """
        Update the camera feed with a new image.

        Parameters
        ----------
        base64_encoded_image
            The base64 encoded string of the image to display.
        """

        pixmap = QPixmap()
        if not pixmap.loadFromData(QByteArray.fromBase64(base64_encoded_image.encode("utf-8"))):
            logger.warning("Failed to decode camera image.")
            return

        self.current_pixmap = pixmap
        self._update_pixmap()

    def _update_pixmap(self) -> None:
        """Scale the current frame to fill the widget while preserving aspect ratio."""

        if self.current_pixmap is None or self.current_pixmap.isNull():
            return

        self.image_label.setPixmap(
            self.current_pixmap.scaled(
                self.image_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def resizeEvent(self, event: QResizeEvent) -> None:
        """Rescale the displayed frame when the widget changes size."""

        super().resizeEvent(event)
        self._update_pixmap()
