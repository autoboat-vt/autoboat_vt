"""
Module containing text input dialog templates.

Functions:
- show_input_dialog: Show an input dialog to get user input.
"""

__all__ = ["show_input_dialog"]

from collections.abc import Callable
from typing import TypeVar

from qtpy.QtWidgets import QInputDialog, QWidget

T = TypeVar("T")


class InputDialog(QWidget):
    """
    A custom input dialog that wraps Qt's QInputDialog.

    Parameters
    ----------
    title
        The title of the dialog window.
    label
        The label text for the input field.
    default_value
        The default value to show in the input field.
    input_type
        The type to convert the input to (int, float, or str).
    parent
        The parent widget for this dialog.
    """

    def __init__(
        self,
        title: str,
        label: str,
        default_value: str | None = None,
        input_type: Callable[[str], T] = str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        self._title = title
        self._label = label
        self._default_value = default_value
        self._input_type = input_type

    def get_input(self) -> T | None:
        """
        Show the input dialog and return the user input converted to the specified type.

        Returns
        -------
        T | None
            The user input converted to the specified type, or ``None`` if the dialog was cancelled.
        """

        if self._input_type is int:
            value = int(self._default_value) if self._default_value is not None else 0
            result, ok = QInputDialog.getInt(self, self.windowTitle(), self._label, value=value)
        elif self._input_type is float:
            value = float(self._default_value) if self._default_value is not None else 0.0
            result, ok = QInputDialog.getDouble(self, self.windowTitle(), self._label, value=value)
        else:
            text = self._default_value if self._default_value is not None else ""
            text, ok = QInputDialog.getText(self, self.windowTitle(), self._label, text=text)
            result = None
            if ok:
                try:
                    result = self._input_type(text)
                except ValueError:
                    print(f"[Error] Failed to convert '{text}' to {self._input_type.__name__}.")
                    return None

        if ok:
            return result if result is not None else text
        return None


def show_input_dialog(
    title: str,
    label: str,
    default_value: str | None = None,
    input_type: Callable[[str], T] = str,
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
        The type to convert the input to. Defaults to ``str``. <br>
        Example: ``int``, ``float``, etc.

    Returns
    -------
    T | None
        The user input converted to the specified type, or ``None`` if the dialog was cancelled.
    """
    
    dialog = InputDialog(title, label, default_value, input_type)
    return dialog.get_input()
