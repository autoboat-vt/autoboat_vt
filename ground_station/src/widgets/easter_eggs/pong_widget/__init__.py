"""
Package housing the Pong easter-egg game and its audio manager.

Exposes:
- PongDialog: A dialog window containing a playable Pong game (1- or 2-player).

Contains:
- `pong_game.py`: Module containing the :class:`PongDialog` and :class:`PongBoard` classes.
"""

__all__ = ["PongDialog"]

from .pong_game import PongDialog
