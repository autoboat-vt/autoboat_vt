from qtpy.QtCore import QRegularExpression
from qtpy.QtGui import QFont, QTextDocument

from utils import constants

from .base_highlighter import BaseHighlighter

__all__ = ["JsonHighlighter"]

class JsonHighlighter(BaseHighlighter):
    """
    A syntax highlighter for JSON text.

    Inherits
    -------
    ``BaseHighlighter``
    """

    def __init__(self, parent: QTextDocument | None = None) -> None:
        super().__init__(parent)

        self.pattern = QRegularExpression(
            r'(?P<key>"(?:\\\\.|[^"\\])*"(?=\s*:))|'
            r'(?P<string>"(?:\\\\.|[^"\\])*")|'
            r"(?P<number>-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?)|"
            r"(?P<keyword>true|false|null)|"
            r"(?P<punct>[{}\[\],:])"
        )

        self.formats = {
            "key": self.create_format(constants.WHITE, QFont.Weight.Normal),
            "string": self.create_format(constants.YELLOW, QFont.Weight.Normal),
            "number": self.create_format(constants.PURPLE, QFont.Weight.Normal),
            "keyword": self.create_format(constants.BLUE, QFont.Weight.Normal),
            "punct": self.create_format(constants.WHITE, QFont.Weight.Normal),
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
