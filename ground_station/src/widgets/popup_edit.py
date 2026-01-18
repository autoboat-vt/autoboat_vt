from qtpy.QtCore import QEvent, QRect, QSize, Qt, Signal
from qtpy.QtGui import (
    QColor,
    QFontDatabase,
    QFontMetrics,
    QPainter,
    QSyntaxHighlighter,
)
from qtpy.QtWidgets import QPlainTextEdit, QPushButton, QVBoxLayout, QWidget


class TextEditWindow(QWidget):
    """
    A simple text edit window that emits the entered text when closed.

    Inherits
    -------
    ``QWidget``

    Parameters
    ----------
    highlighter: ``Optional[QSyntaxHighlighter]``
        An optional syntax highlighter to apply to the text edit area.
        If not provided, no syntax highlighting will be applied.

    initial_text: ``str``
        The initial text to display in the text edit area.
        Default is an empty string.

    tab_width: ``int``
        The width of a tab character in spaces. Default is 4.

    font_size: ``int``
        The font size for the text editor. Default is 14.

    Attributes
    ----------
    user_text_emitter: ``Signal``
        Signal emitted when the window is closed, carrying the entered text.
    """

    user_text_emitter = Signal(str)

    def __init__(
        self,
        highlighter: QSyntaxHighlighter | None = None,
        initial_text: str = "",
        tab_width: int = 4,
        font_size: int = 14,
    ) -> None:
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        self.current_text = initial_text
        self.editor = QPlainTextEdit()
        self.editor.setPlainText(self.current_text)
        self.line_number_area = self.LineNumberArea(self.editor)

        self.tab_width = tab_width
        self.font_size = font_size
        self.highlighter = None
        self._setup_editor()

        if highlighter is not None:
            if issubclass(highlighter, QSyntaxHighlighter):
                self.highlighter = highlighter(self.editor.document())
            else:
                raise TypeError("Highlighter must be a subclass of QSyntaxHighlighter")

        self.save_button = QPushButton("Save (not to file)")
        self.save_button.clicked.connect(self.save)

        self.layout.addWidget(self.editor)
        self.layout.addWidget(self.save_button)

    class LineNumberArea(QWidget):
        """
        A widget to display line numbers in the text editor.

        Inherits
        -------
        ``QWidget``

        Parameters
        ----------
        editor
            The text editor instance to which this line number area belongs.
        """

        def __init__(self, editor: QPlainTextEdit) -> None:
            super().__init__(editor)
            self.editor = editor
            self.text_edit_window = None  # Will be set by parent

        def sizeHint(self) -> QSize:
            """
            Return the size hint for the line number area.

            Returns
            -------
            QSize
                The preferred size of the line number area.
            """

            if self.text_edit_window:
                return QSize(self.text_edit_window.line_number_area_width(), 0)
            return QSize(50, 0)  # Default fallback

        def paintEvent(self, event: QEvent) -> None:
            """
            Handle paint events for the line number area.

            Parameters
            ----------
            event
                The paint event that triggered this method.
            """

            if self.text_edit_window:
                self.text_edit_window.line_number_area_paint_event(event)

    def _setup_editor(self) -> None:
        """
        Set up the text editor with initial configurations.

        This includes setting the font size, tab width, and connecting signals
        for line number area updates.
        """

        self.line_number_area.text_edit_window = self

        font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        font.setPointSize(self.font_size)
        self.editor.setFont(font)
        self.editor.setLineWrapMode(QPlainTextEdit.NoWrap)

        font_metrics = QFontMetrics(font)
        space_width = font_metrics.horizontalAdvance(" ")
        self.editor.setTabStopDistance(self.tab_width * space_width)

        self.editor.blockCountChanged.connect(self.update_line_number_area_width)
        self.editor.updateRequest.connect(self.update_line_number_area)
        self.update_line_number_area_width()

    def line_number_area_width(self) -> int:
        """
        Calculate width needed for line numbers.

        Returns
        -------
        int
            The width in pixels required for the line number area.
        """

        digits = 1
        max_line = max(1, self.editor.blockCount())

        while max_line >= 10:
            max_line //= 10
            digits += 1

        return 10 + self.fontMetrics().horizontalAdvance("9") * digits

    def update_line_number_area_width(self) -> None:
        """Update margins to make space for line numbers."""

        self.editor.setViewportMargins(self.line_number_area_width(), 0, 0, 0)

    def update_line_number_area(self, rect: QRect, dy: int) -> None:
        """
        Update line number area when scrolling.

        Parameters
        ----------
        rect
            The rectangle area that needs to be updated.

        dy
            The vertical scroll offset. If 0, the area is updated without scrolling.
        """

        if dy:
            self.line_number_area.scroll(0, dy)
        else:
            self.line_number_area.update(0, rect.y(), self.line_number_area.width(), rect.height())

        if rect.contains(self.editor.viewport().rect()):
            self.update_line_number_area_width()

    def resizeEvent(self, event: QEvent) -> None:
        """
        Handle editor resize events.

        Parameters
        ----------
        event
            The resize event that triggered this method.
        """

        super().resizeEvent(event)
        cr = self.editor.contentsRect()
        self.line_number_area.setGeometry(QRect(cr.left(), cr.top(), self.line_number_area_width(), cr.height()))

    def line_number_area_paint_event(self, event: QEvent) -> None:
        """
        Paint line numbers in the dedicated area.

        Parameters
        ----------
        event
            The paint event that triggered this method.
        """

        painter = QPainter(self.line_number_area)
        painter.fillRect(event.rect(), QColor("#313131"))

        block = self.editor.firstVisibleBlock()
        block_number = block.blockNumber()
        top = self.editor.blockBoundingGeometry(block).translated(self.editor.contentOffset()).top()
        bottom = top + self.editor.blockBoundingRect(block).height()

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                painter.setPen(QColor("#ffffff"))
                painter.drawText(
                    0,
                    int(top),
                    self.line_number_area.width() - 5,
                    int(self.editor.fontMetrics().height()),
                    Qt.AlignRight,
                    str(block_number + 1),
                )

            block = block.next()
            top = bottom
            bottom = top + self.editor.blockBoundingRect(block).height()
            block_number += 1

    def set_font_size(self, size: int) -> None:
        """
        Update font size dynamically.

        Parameters
        ----------
        size
            The new font size to set for the text editor.
        """

        self.font_size = size
        font = self.editor.font()
        font.setPointSize(size)
        self.editor.setFont(font)

        font_metrics = QFontMetrics(font)
        space_width = font_metrics.horizontalAdvance(" ")
        self.editor.setTabStopDistance(self.tab_width * space_width)

        if self.highlighter:
            self.highlighter.rehighlight()

    def save(self) -> None:
        """Save current text."""

        self.current_text = self.editor.toPlainText()

    def closeEvent(self, event: QEvent) -> None:
        """
        Handle window closing.

        Parameters
        ----------
        event
            The close event that triggered this method.
        """

        self.current_text = self.editor.toPlainText()
        self.user_text_emitter.emit(self.current_text)
        event.accept()
