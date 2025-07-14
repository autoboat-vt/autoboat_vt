from qtpy.QtGui import QSyntaxHighlighter, QTextCharFormat, QFont, QColor


class BaseHighlighter(QSyntaxHighlighter):
    """
    Base class for syntax highlighters.

    Inherits
    --------
    `QSyntaxHighlighter`
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

    @staticmethod
    def create_format(color: QColor, weight: QFont.Weight) -> QTextCharFormat:
        """
        Create a `QTextCharFormat` with the specified color and font weight.

        Parameters
        ----------
        color
            The color to set for the text format.
        weight
            The font weight to set for the text format.

        Returns
        -------
        `QTextCharFormat`
            A text format with the specified color and font weight.
        """

        fmt = QTextCharFormat()
        fmt.setForeground(color)
        fmt.setFontWeight(weight)
        return fmt

    def highlightBlock(self, text: str) -> None:
        """
        Highlight the text block using the defined patterns and formats.

        Parameters
        ----------
        text
            The text block to highlight.

        Raises
        -------
        NotImplementedError
        """

        raise NotImplementedError("Subclasses must implement highlightBlock()")
