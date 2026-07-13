"""
Package housing the Tetris easter-egg game and its audio manager.

Exposes:
- TetrisDialog: A dialog window containing a playable Tetris game.

Contains:
- `tetris_game.py`: Module containing the ``TetrisDialog``, ``TetrisBoard``, and ``Tetromino`` classes.
- `tetris_audio.py`: Module containing the ``TetrisAudio`` class for music and sound-effect playback.
"""

__all__ = ["TetrisDialog"]

from .tetris_game import TetrisDialog
