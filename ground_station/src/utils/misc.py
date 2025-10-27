"""
Module containing miscellaneous utility functions and classes for the ground station application.
"""

import qtawesome as qta
from qtpy.QtWidgets import QPushButton, QMessageBox, QInputDialog, QCheckBox
from qtpy.QtCore import QTimer
from qtpy.QtGui import QIcon
from types import SimpleNamespace
from typing import TypeVar
from collections.abc import Callable

T = TypeVar("T")


def get_icons() -> SimpleNamespace:
    """
    Load and return a set of icons for the application.

    Returns
    -------
    SimpleNamespace
        A namespace object containing the loaded icons.
        Each icon can be accessed as an attribute of the namespace.

        >>> icons.upload == icons["upload"]
        True

    Notes
    -----
    The icons cannot be loaded until a `QApplication` instance is created.

    Raises
    -------
    TypeError
        If an icon fails to load or is not a QIcon.
        This indicates that the icon name is not valid or the icon could not be found.
    """

    icons: dict[str, QIcon] = {
        "upload": qta.icon("mdi.upload", color="white"),
        "download": qta.icon("mdi.download", color="white"),
        "connect": qta.icon("mdi.connection", color="white"),
        "disconnect": qta.icon("fa6s.plug-circle-xmark", color="white"),
        "delete": qta.icon("mdi.trash-can", color="white"),
        "add": qta.icon("mdi.plus", color="white"),
        "save": qta.icon("mdi.content-save", color="white"),
        "cog": qta.icon("mdi.cog", color="white"),
        "pencil": qta.icon("ei.pencil", color="white"),
        "refresh": qta.icon("mdi.refresh", color="white"),
        "hard_drive": qta.icon("fa6.hard-drive", color="white"),
        "boat": qta.icon("mdi.sail-boat", color="white"),
        "image_upload": qta.icon("mdi.image-move", color="white"),
        "notification": qta.icon("mdi.bell", color="white"),
        "warning": qta.icon("mdi.alert", color="white"),
        "question": qta.icon("mdi.help-circle", color="white"),
    }

    for icon_name, icon in icons.items():
        assert isinstance(icon, QIcon), (
            f"Icon '{icon_name}' is not a valid QIcon. Please check the icon name or ensure the icon is available."
        )

    return SimpleNamespace(**icons)


def pushbutton_maker(
    button_text: str,
    icon: QIcon,
    function: Callable[[], None],
    style_sheet: str | None = None,
    max_width: int | None = None,
    min_height: int | None = None,
    is_clickable: bool = True,
) -> QPushButton:
    """
    Create a `QPushButton` with the specified features.

    Parameters
    ----------
    button_text
        The text to display on the button.
    icon
        The icon to display on the button.
    function
        The function to connect to the button's clicked signal.
    style_sheet
        An optional style sheet to apply to the button. If not specified, the default style is used.
    max_width
        The maximum width of the button. If not specified, not used.
    min_height
        The minimum height of the button. If not specified, not used.
    is_clickable
        Whether the button should be clickable. Defaults to `True`.

    Returns
    -------
    QPushButton
        The created button.
    """

    try:
        button = QPushButton(button_text)
        button.setIcon(icon)
        button.clicked.connect(function)

        if style_sheet is not None:
            button.setStyleSheet(style_sheet)

        if max_width is not None:
            button.setMaximumWidth(max_width)

        if min_height is not None:
            button.setMinimumHeight(min_height)

        button.setDisabled(not is_clickable)

    except Exception as e:
        raise RuntimeError(f"Failed to create button '{button_text}': {e}") from e

    return button


def show_message_box(
    title: str,
    message: str,
    icon: QIcon | None = None,
    buttons: list[QMessageBox.StandardButton] | None = None,
    remember_choice_option: bool | None = False,
) -> QMessageBox.StandardButton | tuple[QMessageBox.StandardButton, bool]:
    """
    Show a message box with the specified title, message, and optional icon and buttons.
    Returns the button that was clicked by the user. If the user closes the message box
    without clicking a button, it returns `QMessageBox.StandardButton.NoButton`.

    Parameters
    ----------
    title
        The title of the message box.
    message
        The message to display in the message box.
    icon
        An optional icon to display in the message box.
    buttons
        A list of standard buttons to show. Defaults to `[QMessageBox.Ok]`. <br>
        Example: `[QMessageBox.Yes, QMessageBox.No]`
    remember_choice_option
        If `True`, adds a "Remember my choice" checkbox to the message box.

    Returns
    -------
    QMessageBox.StandardButton
        The button that was clicked by the user.
    bool
        If `remember_choice_option` is `True`, returns whether the user checked the "Remember my choice" checkbox.
    """

    if buttons is None:
        buttons = [QMessageBox.Ok]

    msg_box = QMessageBox()
    msg_box.setWindowTitle(title)
    msg_box.setText(message)

    if icon:
        msg_box.setIconPixmap(icon.pixmap(64, 64))

    for button in buttons:
        msg_box.addButton(button)

    if remember_choice_option:
        remember_checkbox = QCheckBox("Remember my decision")
        msg_box.setCheckBox(remember_checkbox)
        clicked_button = msg_box.exec_()
        return QMessageBox.StandardButton(clicked_button), remember_checkbox.isChecked()

    else:
        clicked_button = msg_box.exec_()
        return QMessageBox.StandardButton(clicked_button)


def show_input_dialog(
    title: str, label: str, default_value: str | None = None, input_type: Callable[[str], T] = str
) -> T | None:
    """
    Show an input dialog to get user input.

    Parameters
    ----------
    title
        The title of the input dialog.
    label
        The label for the input field.
    default_value
        The default value to show in the input field.
    input_type
        The type to convert the input to. Defaults to `str`. <br>
        Example: `int`, `float`, etc.

    Returns
    -------
    T | None
        The user input converted to the specified type, or `None` if the dialog was cancelled.
    """

    if default_value is not None:
        text, ok = QInputDialog.getText(None, title, label, text=default_value)

    else:
        text, ok = QInputDialog.getText(None, title, label)

    if ok:
        try:
            converted_value: T = input_type(text)

        except ValueError:
            print(f"[Error] Failed to convert {text} to {input_type}. Returning None.")
            return None

        return converted_value

    else:
        return None


def create_timer(interval_ms: int, single_shot: bool = False) -> QTimer:
    """
    Create a QTimer with the specified interval and single-shot status.

    Parameters
    ----------
    interval_ms
        The interval in milliseconds for the timer.
    single_shot
        Whether the timer should be single-shot. Defaults to `False`.

    Returns
    -------
    QTimer
        A new QTimer instance with the specified interval and single-shot status.
    """

    timer = QTimer()
    timer.setInterval(interval_ms)
    timer.setSingleShot(single_shot)
    return timer


def copy_qtimer(original: QTimer) -> QTimer:
    """
    Create a copy of a QTimer with the same interval and single-shot status but without copying connections.

    Parameters
    ----------
    original
        The original QTimer to copy.

    Returns
    -------
    QTimer
        A new QTimer instance with the same interval and single-shot status as the original.
    """

    new_timer = QTimer()
    new_timer.setInterval(original.interval())
    new_timer.setSingleShot(original.isSingleShot())
    return new_timer
