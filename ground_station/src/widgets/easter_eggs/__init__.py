"""
Package housing the easter-egg games.

Exposes:
- TetrisDialog: A dialog window containing a playable Tetris game.
- SnakeDialog: A dialog window containing a playable Snake game.
- PongDialog: A dialog window containing a playable Pong game (1- or 2-player).

Contains:
- `tetris_widget/`: The Tetris game and its audio manager.
- `snake_widget/`: The Snake game.
- `pong_widget/`: The Pong game.
"""

__all__ = ["PongDialog", "SnakeDialog", "TetrisDialog"]

from .pong_widget import PongDialog
from .snake_widget import SnakeDialog
from .tetris_widget import TetrisDialog
