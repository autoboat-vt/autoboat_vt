"""
Package for the camera widget in the Groundstation application.

Exposes:
- CameraWidget: The main widget class for displaying the camera feed and related controls.

Contains:
- `camera.py`: The module containing the implementation of the ``CameraWidget`` class.
- `camera.html`: An HTML template used for rendering the camera feed within the widget.
"""

__all__ = ["CameraWidget"]

from .camera import CameraWidget
