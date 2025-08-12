import constants
import thread_classes
import json

from qtpy.QtWebEngineWidgets import QWebEngineView
from qtpy.QtWidgets import QWidget, QGridLayout, QHBoxLayout, QPushButton


class CameraWidget(QWidget):
    """
    A widget to display a camera feed using `QWebEngineView`.

    Inherits
    -------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()
        self.main_layout = QGridLayout()

        self.controls_layout = QHBoxLayout()

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.pause_timer)
        self.is_paused = True
        self.paused_icon_base64 = open(constants.ASSETS_DIR / "paused-icon-base64.txt").read()
        self.pause_button.setDisabled(not self.is_paused)

        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self.unpause_timer)
        self.is_running = False

        self.controls_layout.addWidget(self.pause_button)
        self.controls_layout.addWidget(self.run_button)
        self.main_layout.addLayout(self.controls_layout, 1, 0)

        self.web_view_layout = QHBoxLayout()

        self.web_view = QWebEngineView()
        self.web_view.setHtml(constants.HTML_CAMERA)

        self.web_view_layout.addWidget(self.web_view)
        self.main_layout.addLayout(self.web_view_layout, 0, 0)
        self.setLayout(self.main_layout)

        self.image_fetcher = thread_classes.ImageFetcher()
        self.image_fetcher.image_fetched.connect(self.update_camera_feed)

        self.timer = constants.HALF_SECOND_TIMER
        self.timer.timeout.connect(self.image_fetcher.get_image)

    def unpause_timer(self) -> None:
        """Unpause the timer that fetches images from the camera."""

        self.timer.start()
        self.is_running = True
        self.is_paused = False
        self.run_button.setDisabled(self.is_running)
        self.pause_button.setDisabled(self.is_paused)
        print("[Info] Unpaused camera feed timer.")

    def pause_timer(self) -> None:
        """Pause the timer that fetches images from the camera."""

        self.timer.stop()
        self.is_running = False
        self.is_paused = True
        js_image_str = json.dumps(self.paused_icon_base64)
        self.web_view.page().runJavaScript(f"setBase64Image({js_image_str});")
        self.pause_button.setDisabled(self.is_paused)
        self.run_button.setDisabled(self.is_running)
        print("[Info] Paused camera feed timer.")

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

        js_image_str = json.dumps(base64_encoded_image)

        self.web_view.page().runJavaScript(f"setBase64Image({js_image_str});")
