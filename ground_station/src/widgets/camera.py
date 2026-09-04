from __future__ import annotations

from urllib.parse import urljoin

from qtpy.QtCore import QByteArray, Qt, Slot
from qtpy.QtGui import QPixmap, QResizeEvent
from qtpy.QtWidgets import QFileDialog, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QWidget

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
        self.paused_image = open(constants.ASSETS_DIR / "new_logo.png", "rb").read()
        self.pause_button.setDisabled(not self.is_paused)

        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self.unpause_timer)
        self.is_running = False

        self.upload_button = QPushButton("Upload Image")
        self.upload_button.clicked.connect(self.upload_image)

        self.controls_layout.addWidget(self.pause_button)
        self.controls_layout.addWidget(self.run_button)
        self.controls_layout.addWidget(self.upload_button)
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
        self.timer.timeout.connect(self.update_camera_feed_starter)

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
        self.update_camera_feed(self.paused_image)
        self.pause_button.setDisabled(self.is_paused)
        self.run_button.setDisabled(self.is_running)
        logger.info("Paused camera feed timer.")

    def update_camera_feed_starter(self) -> None:
        """Start the image fetcher thread to update the camera feed if it is not already running."""

        if not self.image_fetcher.isRunning():
            self.image_fetcher.start()

    def update_camera_feed(self, image: bytes) -> None:
        """
        Update the camera feed with a new image.

        Parameters
        ----------
        image
            The new image data.
        """

        pixmap = QPixmap()

        try:
            pixmap.loadFromData(QByteArray(image))

        except Exception as e:
            logger.error(f"Failed to load image from data: {e}")
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

    @Slot()
    def upload_image(self) -> None:
        """
        Open a file dialog to select an image and upload it to the telemetry server.

        Raises
        ------
        :class:`ValueError`
            If the image upload fails.
        """

        file_path, _ = QFileDialog.getOpenFileName(
            parent=self,
            caption="Select an image to upload",
            filter="Image Files (*.png *.jpg *.jpeg *.bmp *.gif)",
            directory=constants.ASSETS_DIR.as_posix()
        )

        if not file_path:
            logger.info("No image selected for upload.")
            return

        try:
            with open(file_path, "rb") as f:
                image_data = f.read()

            response = constants.REQ_SESSION.post(
                urljoin(
                    misc.get_route("set_current_image"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                ),
                files={"image": image_data},
            )

            if response.status_code == 200:
                logger.info(f"Successfully uploaded image: {file_path}")
            else:
                raise ValueError(response.text.strip())

        except Exception as e:
            logger.error(f"Failed to upload image: {e}")
