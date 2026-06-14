from __future__ import annotations

__all__ = ["PreserveAspectWidget"]

from qtpy.QtGui import QResizeEvent
from qtpy.QtWidgets import QWidget


class PreserveAspectWidget(QWidget):
    """
    Container that preserves the aspect ratio of a child widget.

    Parameters
    ----------
    child
        The ``QWidget`` to resize inside this container.
    aspect_ratio
        The target width divided by height.

    Raises
    ------
    ValueError
        If ``aspect_ratio`` is not greater than zero.

    Inherits
    --------
    ``QWidget``
    """

    def __init__(self, child: QWidget, aspect_ratio: float = 1.0) -> None:
        super().__init__()

        if aspect_ratio <= 0:
            raise ValueError("aspect_ratio must be greater than zero.")

        self.child = child
        self.aspect_ratio = aspect_ratio

        self.child.setParent(self)

    def resizeEvent(self, event: QResizeEvent) -> None:
        """
        Resize the child widget while preserving its aspect ratio.

        Parameters
        ----------
        event
            The resize event.
        """

        container_width = self.width()
        container_height = self.height()

        child_width = container_width
        child_height = int(child_width / self.aspect_ratio)

        if child_height > container_height:
            child_height = container_height
            child_width = int(child_height * self.aspect_ratio)

        x = (container_width - child_width) // 2
        y = (container_height - child_height) // 2

        self.child.setGeometry(x, y, child_width, child_height)

        super().resizeEvent(event)
