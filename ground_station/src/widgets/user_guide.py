from qtpy.QtGui import QCloseEvent
from qtpy.QtWebEngineWidgets import QWebEnginePage, QWebEngineProfile, QWebEngineView
from qtpy.QtWidgets import QVBoxLayout, QWidget

from utils import constants, misc


class UserGuideWidget(QWidget):
    """
    A widget for displaying the documentation.

    Inherits
    -------
    :class:`QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.web_profile = QWebEngineProfile("UserGuide", self)
        self.web_profile.setHttpCacheType(QWebEngineProfile.HttpCacheType.MemoryHttpCache)
        
        self.web_view = QWebEngineView()
        self.web_view.setPage(QWebEnginePage(self.web_profile, self.web_view))
        self.web_view.load(constants.DOCUMENTATION_URL)

        self.home_button = misc.pushbutton_maker("Return to home page", self.navigate_home, constants.ICONS.home)

        self.layout.addWidget(self.web_view)
        self.layout.addWidget(self.home_button)

    def navigate_home(self) -> None:
        """Navigate to the home page of the documentation."""

        self.web_view.load(constants.DOCUMENTATION_URL)

    def closeEvent(self, event: QCloseEvent) -> None:
        """
        Handle the close event for the widget.
        
        Parameters
        ----------
        event
            The close event to be handled.
        """

        self.web_profile.clearHttpCache()
        self.web_profile.clearAllVisitedLinks()
        self.web_profile.clearAllPersistentData()

        super().closeEvent(event)
