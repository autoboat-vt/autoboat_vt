from __future__ import annotations

from urllib.parse import urljoin

from qtpy.QtCore import QByteArray, Qt, Slot
from qtpy.QtGui import QPainter, QPen, QPixmap, QResizeEvent
from qtpy.QtWidgets import QFileDialog, QGridLayout, QHBoxLayout, QLabel, QSizePolicy, QWidget

from utils import constants, misc, thread_classes
from utils.console_logger import get_logger

__all__ = ["CameraWidget"]

logger = get_logger(__name__)


class LoadingSpinner(QWidget):
    """Display a rotating partial ring while the camera feed is buffering."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.angle = 0
        self.setFixedSize(48, 48)
        self.animation_timer = misc.create_timer(50)
        self.animation_timer.timeout.connect(self.rotate)

    def start(self) -> None:
        """Start rotating the spinner."""

        self.show()
        self.animation_timer.start()

    def stop(self) -> None:
        """Stop rotating and hide the spinner."""

        self.animation_timer.stop()
        self.hide()

    def rotate(self) -> None:
        """Advance the spinner rotation."""

        self.angle = (self.angle + 30) % 360
        self.update()

    def paintEvent(self, _event: object) -> None:
        """Paint the rotating partial ring."""

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(Qt.GlobalColor.white, 4)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen)
        painter.drawArc(self.rect().adjusted(6, 6, -6, -6), self.angle * 16, 120 * 16)


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

        self.run_button = misc.pushbutton_maker(
            button_text="Start",
            function=self.start_timer,
            icon=constants.ICONS.play_circle_outline,
            min_height=30,
            is_clickable=True,
            tooltip="Resume the camera feed timer",
        )

        self.pause_button = misc.pushbutton_maker(
            button_text="Pause",
            function=self.pause_timer,
            icon=constants.ICONS.stop_circle_outline,
            min_height=30,
            is_clickable=False,
            tooltip="Pause the camera feed timer",
        )

        self.upload_button = misc.pushbutton_maker(
            button_text="Upload Image",
            function=self.upload_image,
            icon=constants.ICONS.upload,
            min_height=30,
            is_clickable=True,
            tooltip="Upload an image to the telemetry server",
        )

        self.controls_layout.addWidget(self.run_button)
        self.controls_layout.addWidget(self.pause_button)
        self.controls_layout.addWidget(self.upload_button)
        self.main_layout.addLayout(self.controls_layout, 1, 0)

        self.web_view_layout = QHBoxLayout()

        self.current_pixmap: QPixmap | None = None
        self.frame_count = 0
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("background-color: black;")

        self.frame_rate_label = QLabel("FPS: 0", parent=self.image_label)
        self.frame_rate_label.setStyleSheet(
            "color: white; background-color: rgba(0, 0, 0, 180); padding: 4px;"
        )
        self.frame_rate_label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.frame_rate_label.adjustSize()
        self.frame_rate_label.raise_()

        self.image_status_label = QLabel("Click the 'Start' button to activate the camera feeda", parent=self.image_label)
        self.image_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_status_label.setStyleSheet("color: white; font-size: 24px; font-weight: bold;")
        self.image_status_label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.image_status_label.raise_()
        self.buffering_spinner = LoadingSpinner(self.image_status_label)
        self.buffering_spinner.hide()
        self._position_image_status_label()

        self.image_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.image_label.setMinimumSize(1, 1)

        self.web_view_layout.addWidget(self.image_label)
        self.main_layout.addLayout(self.web_view_layout, 0, 0)
        self.setLayout(self.main_layout)

        self.image_fetcher = thread_classes.ImageThreadRouter.ImageFetcher()
        self.image_fetcher.data_fetched.connect(self.update_camera_feed)

        self.timer = misc.copy_qtimer(constants.ONE_MS_TIMER)
        self.timer.timeout.connect(self.update_camera_feed_starter)

        self.no_image_timer = misc.create_timer(3_000, single_shot=True)
        self.no_image_timer.timeout.connect(self.show_no_image_message)

        self.frame_rate_timer = misc.copy_qtimer(constants.ONE_SECOND_TIMER)
        self.frame_rate_timer.timeout.connect(self.update_frame_rate)
        self.frame_rate_timer.start()

    def pause_timer(self) -> None:
        """Pause the timer that fetches images from the camera."""

        self.timer.stop()
        self.no_image_timer.stop()
        self.buffering_spinner.stop()
        self.image_status_label.hide()
        self.pause_button.setDisabled(True)
        self.run_button.setDisabled(False)
        logger.info("Paused camera feed timer.")

    def start_timer(self) -> None:
        """Start the timer that fetches images from the camera."""

        self.timer.start()
        self.image_status_label.setText("")
        self.image_status_label.show()
        self.image_status_label.raise_()
        self.buffering_spinner.start()
        self._position_image_status_label()
        self.no_image_timer.start()
        self.run_button.setDisabled(True)
        self.pause_button.setDisabled(False)
        logger.info("Unpaused camera feed timer.")

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
        loaded = pixmap.loadFromData(QByteArray(image))

        if loaded and image != b"":
            self.no_image_timer.stop()
            self.buffering_spinner.stop()
            self.image_status_label.hide()
            self.current_pixmap = pixmap
            self.frame_count += 1
            self._update_pixmap()

    def show_no_image_message(self) -> None:
        """Show the fallback message when buffering takes too long."""

        self.buffering_spinner.stop()
        self.image_status_label.setText("No image available")
        self.image_status_label.show()
        self.image_status_label.raise_()
        self._position_image_status_label()

    def update_frame_rate(self) -> None:
        """Update the displayed frame rate once per second."""

        self.frame_rate_label.setText(f"FPS: {self.frame_count}")
        self.frame_count = 0
        self._position_frame_rate_label()

    def _position_frame_rate_label(self) -> None:
        """Keep the frame rate label anchored to the top-left of the video area."""

        self.frame_rate_label.adjustSize()
        self.frame_rate_label.move(8, 8)

    def _position_image_status_label(self) -> None:
        """Keep the image status label centered in the video area."""

        self.image_status_label.setGeometry(self.image_label.rect())
        self.buffering_spinner.move(
            (self.image_status_label.width() - self.buffering_spinner.width()) // 2,
            (self.image_status_label.height() - self.buffering_spinner.height()) // 2,
        )

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
        self._position_frame_rate_label()
        self._position_image_status_label()

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
