from __future__ import annotations

from qtpy.QtGui import QResizeEvent
from qtpy.QtWidgets import QWidget


class BoundedAspectWidget(QWidget):
    """
    Container that keeps a child widget within an aspect ratio range.

    Parameters
    ----------
    child
        The ``QWidget`` to resize inside this container.
    min_aspect_ratio
        The minimum width divided by height.
    max_aspect_ratio
        The maximum width divided by height.

    Raises
    ------
    ValueError
        - If ``min_aspect_ratio`` is not greater than zero.
        - If ``max_aspect_ratio`` is less than ``min_aspect_ratio``.

    Inherits
    --------
    ``QWidget``
    """

    def __init__(self, child: QWidget, min_aspect_ratio: float, max_aspect_ratio: float) -> None:
        super().__init__()

        if min_aspect_ratio <= 0:
            raise ValueError("min_aspect_ratio must be greater than zero.")

        if max_aspect_ratio < min_aspect_ratio:
            raise ValueError("max_aspect_ratio must be greater than or equal to min_aspect_ratio.")

        self.child = child
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio

        self.child.setParent(self)

    def resizeEvent(self, event: QResizeEvent) -> None:
        """
        Resize the child widget while keeping it within the aspect ratio range.

        Parameters
        ----------
        event
            The resize event.
        """

        container_width = self.width()
        container_height = self.height()

        if container_width <= 0 or container_height <= 0:
            super().resizeEvent(event)
            return

        aspect_ratio = container_width / container_height

        child_width = container_width
        child_height = container_height

        if aspect_ratio > self.max_aspect_ratio:
            child_width = int(container_height * self.max_aspect_ratio)

        elif aspect_ratio < self.min_aspect_ratio:
            child_height = int(container_width / self.min_aspect_ratio)

        x = (container_width - child_width) // 2
        y = (container_height - child_height) // 2

        self.child.setGeometry(x, y, child_width, child_height)

        super().resizeEvent(event)
