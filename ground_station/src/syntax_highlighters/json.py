from qtpy.QtCore import QRegularExpression
from qtpy.QtGui import QFont
from typing import Optional
from constants import WHITE, YELLOW, PURPLE, BLUE
from syntax_highlighters.base_highlighter import BaseHighlighter


class JsonHighlighter(BaseHighlighter):
    """
    A syntax highlighter for JSON text.

    Inherits
    -------
    `BaseHighlighter`
    """

    def __init__(self, parent: Optional[object] = None) -> None:
        super().__init__(parent)

        self.pattern = QRegularExpression(
            r'(?P<key>"(?:\\\\.|[^"\\])*"(?=\s*:))|'
            r'(?P<string>"(?:\\\\.|[^"\\])*")|'
            r"(?P<number>-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?)|"
            r"(?P<keyword>true|false|null)|"
            r"(?P<punct>[{}\[\],:])"
        )

        self.formats = {
            "key": self.create_format(WHITE, QFont.Normal),
            "string": self.create_format(YELLOW, QFont.Normal),
            "number": self.create_format(PURPLE, QFont.Normal),
            "keyword": self.create_format(BLUE, QFont.Normal),
            "punct": self.create_format(WHITE, QFont.Normal),
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
