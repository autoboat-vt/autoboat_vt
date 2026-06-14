"""
Module containing custom message dialog templates.

Functions:
- show_message_box: Show a message box with specified title, message, icon, and buttons.
- show_custom_message_box: Show a custom message dialog with custom buttons.
"""

__all__ = ["MessageBoxButton", "show_custom_message_box", "show_message_box"]

from typing import NamedTuple

from qtpy.QtGui import QIcon
from qtpy.QtWidgets import (
    QAbstractButton,
    QCheckBox,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)

from utils import constants

from .base_dialog import BaseDialog


class MessageBoxButton(NamedTuple):
    """
    Custom button definition for message boxes.
    
    Inherits
    --------
    ``NamedTuple``
    """

    key: str
    text: str
    role: QMessageBox.ButtonRole


class CustomMessageBoxDialog(BaseDialog):
    """
    A custom message dialog with custom buttons and optional "remember choice" checkbox.

    Parameters
    ----------
    title
        The title of the dialog window.
    message
        The message to display in the dialog.
    icon
        An optional icon to display next to the message.
    buttons
        A list of ``MessageBoxButton`` tuples defining the buttons.
    remember_choice_option
        Whether to show a "Remember my decision?" checkbox.

    Attributes
    ----------
    user_text_emitter: ``Signal``
        Emitted when the dialog is closed with a button press, passing the button key.

    Inherits
    -------
    ``BaseDialog``
    """

    def __init__(
        self,
        title: str,
        message: str,
        icon: QIcon | None = None,
        buttons: list[MessageBoxButton] | None = None,
        remember_choice_option: bool = False,
    ) -> None:
        super().__init__(title=title, message=message, icon=icon, remember_choice_option=remember_choice_option)

        self.set_result(buttons[-1].key if buttons else "")

        button_layout = QVBoxLayout()

        for button_info in buttons or []:
            button = QPushButton(button_info.text)
            button.clicked.connect(
                lambda _checked=False, key=button_info.key: (
                    self.set_result(key),
                    self.accept(),
                )
            )
            button_layout.addWidget(button)

        self.layout().addLayout(button_layout)

    def get_result_with_remember(self) -> tuple[str, bool]:
        """Return the button key and checkbox state."""
        return self.get_result(), self.get_remember_choice()


def show_message_box(
    title: str,
    message: str,
    icon: QIcon | None = None,
    buttons: list[QMessageBox.StandardButton | MessageBoxButton] | None = None,
    remember_choice_option: bool | None = False,
) -> QMessageBox.StandardButton | str | tuple[QMessageBox.StandardButton | str, bool]:
    """
    Show a message box with the specified title, message, icon, and buttons.

    Parameters
    ----------
    title
        The title of the message box.
    message
        The message to display in the message box.
    icon
        An optional icon to display in the message box.
    buttons
        An optional list of buttons to display in the message box. Supports either
        standard buttons or custom MessageBoxButton values. Do not mix both types.
        Defaults to a single OK button.
    remember_choice_option
        Whether to include a "Remember my decision?" checkbox.

    Returns
    -------
    QMessageBox.StandardButton | str | tuple[QMessageBox.StandardButton | str, bool]
        The selected standard button, custom button key, or the same value with
        the checkbox state when remember_choice_option is True.

    Raises
    ------
    TypeError
        If both standard buttons and custom buttons are included.
    """

    if buttons is None:
        buttons = [QMessageBox.StandardButton.Ok]

    msg_box = QMessageBox()
    msg_box.setWindowTitle(title)
    msg_box.setText(message)

    if icon:
        msg_box.setIconPixmap(icon.pixmap(64, 64))

    uses_custom_buttons = any(isinstance(button, MessageBoxButton) for button in buttons)
    uses_standard_buttons = any(isinstance(button, QMessageBox.StandardButton) for button in buttons)

    if uses_custom_buttons and uses_standard_buttons:
        raise TypeError("Do not mix StandardButton and MessageBoxButton in one message box.")

    custom_button_map: dict[QAbstractButton, str] = {}

    if uses_custom_buttons:
        for button in buttons:
            if not isinstance(button, MessageBoxButton):
                raise TypeError("Expected MessageBoxButton.")

            added_button = msg_box.addButton(button.text, button.role)
            custom_button_map[added_button] = button.key

        longest_button_text = max(
            msg_box.fontMetrics().horizontalAdvance(btn.text) for btn in buttons if isinstance(btn, MessageBoxButton)
        )

        min_width = min(
            constants.MAX_WINDOW_SIZE.width() // 2,
            max(500, longest_button_text + 160),
        )

        msg_box.setMinimumWidth(min_width)

    else:
        std_buttons = QMessageBox.StandardButton.NoButton

        for button in buttons:
            if not isinstance(button, QMessageBox.StandardButton):
                raise TypeError("Expected QMessageBox.StandardButton.")

            std_buttons |= button

        msg_box.setStandardButtons(std_buttons)

    remember_checkbox = None

    if remember_choice_option:
        remember_checkbox = QCheckBox("Remember my decision?")
        msg_box.setCheckBox(remember_checkbox)

    clicked = msg_box.exec()

    if uses_custom_buttons:
        clicked_button = msg_box.clickedButton()
        response = custom_button_map.get(clicked_button)

        if response is None:
            fallback_button = buttons[-1]
            if not isinstance(fallback_button, MessageBoxButton):
                raise TypeError("Expected MessageBoxButton fallback.")

            response = fallback_button.key
            print(f"[Warning] User closed the dialog without selecting a button. Using {response}.")

    else:
        response = QMessageBox.StandardButton(clicked)

        if response == QMessageBox.StandardButton.NoButton:
            fallback_button = buttons[-1]
            if not isinstance(fallback_button, QMessageBox.StandardButton):
                raise TypeError("Expected QMessageBox.StandardButton fallback.")

            response = fallback_button
            print(f"[Warning] User closed the dialog without selecting a button. Using {response}.")

    if remember_choice_option:
        return response, remember_checkbox.isChecked()

    return response


def show_custom_message_box(
    title: str,
    message: str,
    icon: QIcon | None,
    buttons: list[MessageBoxButton],
    remember_choice_option: bool = False,
) -> str | tuple[str, bool]:
    """
    Show a custom message dialog with custom buttons.

    Parameters
    ----------
    title
        The title of the dialog.
    message
        The message to display in the dialog.
    icon
        An optional icon to display next to the message.
    buttons
        A list of ``MessageBoxButton`` tuples defining the buttons.
    remember_choice_option
        Whether to show a "Remember my decision?" checkbox.

    Returns
    -------
    str | tuple[str, bool]
        The selected button key, or the same value with the checkbox state
        when remember_choice_option is True.
    """

    dialog = CustomMessageBoxDialog(title, message, icon, buttons, remember_choice_option)
    dialog.exec()

    if remember_choice_option:
        return dialog.get_result_with_remember()

    return dialog.get_result()
