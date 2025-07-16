import sys
from datetime import datetime

from qtpy.QtCore import QThread, Signal
from qtpy.QtGui import QTextCursor, QCloseEvent
from qtpy.QtWidgets import QWidget, QVBoxLayout, QTextEdit

from syntax_highlighters.console import ConsoleHighlighter


class EmittingStream(QThread):
    """
    A custom stream that emits text written to it as a signal.

    Inherits
    --------
    `QThread`

    Attributes
    ----------
    textWritten: `Signal`
        Signal emitted when text is written to the stream.
    """

    textWritten = Signal(str)

    def write(self, text) -> None:
        """
        Write text to the stream and emit a signal with the text.

        Parameters
        ----------
        text
            The text to write to the stream.
        """

        self.textWritten.emit(str(text))

    def flush(self) -> None:
        """
        Flush the stream. This method is required for compatibility with
        standard output streams.
        """

        pass


class ConsoleOutputWidget(QWidget):
    """
    A widget for displaying console output in a text edit with syntax highlighting.

    Inherits
    --------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setLineWrapMode(QTextEdit.NoWrap)
        self.main_layout.addWidget(self.console_output)

        self.highlighter = ConsoleHighlighter(self.console_output.document())

        self.stdout_stream = EmittingStream()
        self.stderr_stream = EmittingStream()

        sys.stdout = self.stdout_stream
        sys.stderr = self.stderr_stream

        self.stdout_stream.textWritten.connect(self.append_text)
        self.stderr_stream.textWritten.connect(self.append_text)

    def append_text(self, text: str) -> None:
        """
        Append text to the console output widget only.

        Parameters
        ----------
        text
            The text to append to the console output.
        """

        if text.strip():
            now = datetime.now()
            formatted_time = now.strftime("(%I:%M:%S %p)")
            cursor = self.console_output.textCursor()
            cursor.movePosition(QTextCursor.End)
            cursor.insertText(
                f"{formatted_time} {text}\n" + (int(len(text.splitlines()) > 1) * "\n")
            )

            self.console_output.setTextCursor(cursor)
            self.console_output.ensureCursorVisible()

    def closeEvent(self, event: QCloseEvent) -> None:
        """
        Restore original streams when widget is closed.

        Parameters
        ----------
        event
            The close event that triggered this method.
        """

        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr
        super().closeEvent(event)
