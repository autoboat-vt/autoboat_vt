__all__ = ["InputDialog"]

from collections.abc import Callable
from typing import Generic, TypeVar

from qtpy.QtWidgets import QInputDialog, QWidget

from utils.console_logger import get_logger

logger = get_logger(__name__)

T = TypeVar("T")


class InputDialog(QWidget, Generic[T]):
    """
    A dialog for getting user input with a specified type.

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

    Inherits
    --------
    :class:`QWidget` and ``Generic[T]``
    """

    def __init__(
        self,
        title: str,
        label: str,
        default_value: str | None = None,
        input_type: Callable[[str], T] = str,
    ) -> None:
        super().__init__()
        self.setWindowTitle(title)

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
            return result if ok else None

        if self._input_type is float:
            value = float(self._default_value) if self._default_value is not None else 0.0
            result, ok = QInputDialog.getDouble(self, self.windowTitle(), self._label, value=value)
            return result if ok else None

        text = self._default_value if self._default_value is not None else ""
        text, ok = QInputDialog.getText(self, self.windowTitle(), self._label, text=text)
        if not ok:
            return None

        try:
            return self._input_type(text)
        except ValueError:
            logger.error(f"Failed to convert '{text}' to {self._input_type.__name__}.")
            return None
