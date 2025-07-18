from qtpy.QtCore import QRegularExpression
from qtpy.QtGui import QFont
from typing import Optional
from constants import YELLOW, RED, GREEN, WHITE
from syntax_highlighters.base_highlighter import BaseHighlighter


class ConsoleHighlighter(BaseHighlighter):
    """
    A syntax highlighter for console output text.

    Inherits
    -------
    `BaseHighlighter`
    """

    def __init__(self, parent: Optional[object] = None) -> None:
        super().__init__(parent)

        self.pattern = QRegularExpression(
            r"\((?P<timestamp>[^()]+)\)\s*|"
            r"(?P<error>Error)|"
            r"(?P<warning>Warning)|"
            r"(?P<info>Info)"
            r"(?P<text>.*)"
        )

        self.formats = {
            "timestamp": self.create_format(WHITE, QFont.Bold),
            "error": self.create_format(RED, QFont.Bold),
            "warning": self.create_format(YELLOW, QFont.Bold),
            "info": self.create_format(GREEN, QFont.Bold),
            "text": self.create_format(WHITE, QFont.Normal),
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
