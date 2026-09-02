import sys
from datetime import datetime

import pytz

from qtpy.QtCore import QObject, Signal, Slot
from qtpy.QtGui import QCloseEvent, QTextCursor
from qtpy.QtWidgets import QTextEdit, QVBoxLayout, QWidget

from utils import syntax_highlighters
from utils.console_logger import attach_console_widget


class EmittingStream(QObject):
    """
    A custom stream that emits text written to it as a signal.

    This stream is intended to replace ``sys.stdout`` / ``sys.stderr`` so that
    output from :func:`print()` calls throughout the codebase is captured into the
    console widget.

    Attributes
    ----------
    text_written
        Signal emitted when text is written to the stream.

    Inherits
    --------
    :class:`QObject`
    """

    text_written = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.terminal = sys.stdout

    def write(self, text: str) -> None:
        """
        Write text to the stream and emit a signal with the text.

        Parameters
        ----------
        text
            The text to write to the stream.
        """

        self.terminal.write(text)
        self.terminal.flush()
        self.text_written.emit(text)

    def flush(self) -> None:
        """
        Flush the stream. This method is required for compatibility with
        standard output streams.
        """

        self.terminal.flush()


class ConsoleOutputWidget(QWidget):
    """
    A widget for displaying console output in a text edit with syntax highlighting.

    Inherits
    --------
    :class:`QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.main_layout.addWidget(self.console_output)

        self.highlighter = syntax_highlighters.ConsoleHighlighter(self.console_output.document())

        self.stdout_stream = EmittingStream()
        self.stderr_stream = EmittingStream()

        sys.stdout = self.stdout_stream
        sys.stderr = self.stderr_stream

        self.stdout_stream.text_written.connect(self.append_text)
        self.stderr_stream.text_written.connect(self.append_text)

        attach_console_widget(self.append_text)

    @Slot(str)
    def append_text(self, text: str) -> None:
        """
        Append text to the console output widget only.

        Parameters
        ----------
        text
            The text to append to the console output.
        """

        if text.strip():
            now = datetime.now(pytz.timezone("America/New_York"))
            formatted_time = now.strftime("(%I:%M:%S %p)")
            cursor = self.console_output.textCursor()
            cursor.movePosition(QTextCursor.End)
            cursor.insertText(f"{formatted_time} {text}\n" + (int(len(text.splitlines()) > 1) * "\n"))

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
