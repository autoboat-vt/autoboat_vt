from PyQt5.QtCore import QRegularExpression
from PyQt5.QtGui import QFont
from constants import YELLOW, RED, GREEN
from syntax_highlighters.base_highlighter import BaseHighlighter


class ConsoleHighlighter(BaseHighlighter):
    """
    A syntax highlighter for console output text.

    Inherits
    -------
    `BaseHighlighter`
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.pattern = QRegularExpression(
            r"(?P<error>Error:.*)|"
            r"(?P<warning>Warning:.*)|"
            r"(?P<info>Info:.*)"
        )

        self.formats = {
            "error": self.create_format(RED, QFont.Bold),
            "warning": self.create_format(YELLOW, QFont.Normal),
            "info": self.create_format(GREEN, QFont.Normal),
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
