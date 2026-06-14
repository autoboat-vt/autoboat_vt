from __future__ import annotations

from qtpy.QtCore import QRegularExpression
from qtpy.QtGui import QFont, QTextDocument

from utils import constants

from .base_highlighter import BaseHighlighter

__all__ = ["ConsoleHighlighter"]


class ConsoleHighlighter(BaseHighlighter):
    """
    A syntax highlighter for console output text.

    Inherits
    -------
    ``BaseHighlighter``
    """

    def __init__(self, parent: QTextDocument | None = None) -> None:
        super().__init__(parent)

        self.pattern = QRegularExpression(
            r"\((?P<timestamp>[^()]+)\)\s*|"
            r"(?P<error>Error)|"
            r"(?P<warning>Warning)|"
            r"(?P<info>Info)"
            r"(?P<text>.*)"
        )

        self.formats = {
            "timestamp": self.create_format(constants.WHITE, QFont.Weight.Bold),
            "error": self.create_format(constants.RED, QFont.Weight.Bold),
            "warning": self.create_format(constants.YELLOW, QFont.Weight.Bold),
            "info": self.create_format(constants.GREEN, QFont.Weight.Bold),
            "text": self.create_format(constants.WHITE, QFont.Weight.Normal),
        }

    def highlightBlock(self, text: str) -> None:
        """
        Highlight the text block using the defined patterns and formats.

        Parameters
        ----------
        text
            The text block to highlight.
        """

        iterator = self.pattern.globalMatch(text)
        while iterator.hasNext():
            match = iterator.next()

            for name, fmt in self.formats.items():
                start = match.capturedStart(name)
                if start >= 0:
                    length = match.capturedLength(name)
                    self.setFormat(start, length, fmt)
                    break
