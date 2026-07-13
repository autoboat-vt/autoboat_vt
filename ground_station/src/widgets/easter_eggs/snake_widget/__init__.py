"""
Package housing the Snake easter-egg game and its audio manager.

Exposes:
- SnakeDialog: A dialog window containing a playable Snake game.

Contains:
- `snake_game.py`: Module containing the ``SnakeDialog`` and ``SnakeBoard`` classes.
"""

__all__ = ["SnakeDialog"]

from .snake_game import SnakeDialog
