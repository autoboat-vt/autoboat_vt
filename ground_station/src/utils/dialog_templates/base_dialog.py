from qtpy.QtCore import Signal
from qtpy.QtGui import QIcon
from qtpy.QtWidgets import QCheckBox, QDialog, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class BaseDialog(QDialog):
    """
    Base class for dialog widgets.

    Parameters
    ----------
    title
        The window title for the dialog.
    message
        The message text to display.
    icon
        An optional icon to display next to the message.
    remember_choice_option
        Whether to show a "Remember my decision?" checkbox.
    parent
        The parent widget for this dialog.

    Attributes
    ----------
    user_text_emitter: ``Signal``
        Emitted when the dialog is closed with a valid selection, passing
        the result value (typically the button key or selected value).
    dialog_closed: ``Signal``
        Emitted when the dialog is closed, passing whether it was accepted (True) or rejected (False).

    Inherits
    --------
    ``QDialog``
    """

    user_text_emitter = Signal(str)
    dialog_closed = Signal(bool)

    def __init__(
        self,
        title: str | None = None,
        message: str | None = None,
        icon: QIcon | None = None,
        remember_choice_option: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        if title:
            self.setWindowTitle(title)

        self._result: str = ""
        self._remember_choice: bool = False

        self._setup_ui(message, icon, remember_choice_option)

    def _setup_ui(
        self,
        message: str | None,
        icon: QIcon | None,
        remember_choice_option: bool,
    ) -> None:
        """Set up the dialog UI layout and widgets."""

        main_layout = QVBoxLayout(self)

        # Message and icon
        if message is not None:
            if icon is not None:
                top_layout = QHBoxLayout()
                icon_label = QLabel()
                icon_label.setPixmap(icon.pixmap(64, 64))
                message_label = QLabel(message)
                message_label.setWordWrap(True)
                top_layout.addWidget(icon_label)
                top_layout.addWidget(message_label, stretch=1)
                main_layout.addLayout(top_layout)
            else:
                message_label = QLabel(message)
                message_label.setWordWrap(True)
                main_layout.addWidget(message_label)

        # Remember choice checkbox
        self._remember_checkbox: QCheckBox | None = None
        if remember_choice_option:
            self._remember_checkbox = QCheckBox("Remember my decision?")
            main_layout.addWidget(self._remember_checkbox)

    def get_result(self) -> str:
        """Get the dialog result value."""
        return self._result

    def set_result(self, value: str) -> None:
        """Set the dialog result value."""
        self._result = value

    def get_remember_choice(self) -> bool:
        """Get the remember choice checkbox state."""
        return self._remember_choice

    def accept(self) -> None:
        """Override accept to capture checkbox state before accepting."""
        if self._remember_checkbox is not None:
            self._remember_choice = self._remember_checkbox.isChecked()
        super().accept()
        self.dialog_closed.emit(True)

    def reject(self) -> None:
        """Override reject to capture checkbox state before rejecting."""
        if self._remember_checkbox is not None:
            self._remember_choice = self._remember_checkbox.isChecked()
        super().reject()
        self.dialog_closed.emit(False)

    def get_result_with_remember(self) -> tuple[str, bool]:
        """Return a tuple of (result, remember_choice)."""
        return self._result, self._remember_choice
        
        


