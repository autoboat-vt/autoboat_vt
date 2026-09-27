"""
Package for widgets used to control the size of other widgets.

Contains:
- BoundedAspectWidget: A widget that keeps a child widget within a specified aspect ratio range.
- PreserveAspectWidget: A widget that preserves the aspect ratio of a child widget.
"""

__all__ = ["BoundedAspectWidget", "PreserveAspectWidget"]

from .bounded_aspect_widget import BoundedAspectWidget
from .preserve_aspect_widget import PreserveAspectWidget
