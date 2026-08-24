"""Thank you for playing Tetris on the phallic ice cream."""

from __future__ import annotations

import random

from qtpy.QtCore import Qt, QTimer, Signal
from qtpy.QtGui import (
    QCloseEvent,
    QColor,
    QFont,
    QHideEvent,
    QKeyEvent,
    QPainter,
    QPaintEvent,
    QShowEvent,
)
from qtpy.QtWidgets import (
    QDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from .tetris_audio import TetrisAudio

__all__ = ["TetrisDialog"]


# Board dimensions (in blocks).
BOARD_WIDTH: int = 10
BOARD_HEIGHT: int = 20
BLOCK_SIZE: int = 30  # pixels per block

# Each tetromino is a list of (row, col) offsets relative to a pivot.
# We rotate them by transforming (row, col) -> (col, -row).
TETROMINOES: dict[str, list[tuple[int, int]]] = {
    "I": [(0, 0), (0, 1), (0, 2), (0, 3)],
    "O": [(0, 0), (0, 1), (1, 0), (1, 1)],
    "T": [(0, 0), (0, 1), (0, 2), (1, 1)],
    "S": [(0, 1), (0, 2), (1, 0), (1, 1)],
    "Z": [(0, 0), (0, 1), (1, 1), (1, 2)],
    "J": [(0, 0), (1, 0), (1, 1), (1, 2)],
    "L": [(0, 2), (1, 0), (1, 1), (1, 2)],
}

TETROMINO_COLORS: dict[str, QColor] = {
    "I": QColor(0, 240, 240),
    "O": QColor(240, 240, 0),
    "T": QColor(160, 0, 240),
    "S": QColor(0, 240, 0),
    "Z": QColor(240, 0, 0),
    "J": QColor(0, 0, 240),
    "L": QColor(240, 160, 0),
}


class Tetromino:
    """A single falling tetromino piece."""

    def __init__(self, shape: str) -> None:
        self.shape: str = shape
        self.color: QColor = TETROMINO_COLORS[shape]
        self.cells: list[tuple[int, int]] = list(TETROMINOES[shape])
        # spawn near the top-center of the board
        self.row: int = 0
        self.col: int = BOARD_WIDTH // 2 - 2

    def absolute_cells(self) -> list[tuple[int, int]]:
        """Return the (row, col) positions of this piece on the board."""

        return [(self.row + dr, self.col + dc) for dr, dc in self.cells]

    def rotated_cells(self) -> list[tuple[int, int]]:
        """Return the cells as they would be after a clockwise rotation."""
        # (row, col) -> (col, -row) is a 90-degree clockwise rotation
        return [(dc, -dr) for dr, dc in self.cells]


class TetrisBoard(QFrame):
    """The playable Tetris board widget."""

    lines_cleared_signal = Signal(int)
    game_over_signal = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.setMinimumSize(BOARD_WIDTH * 16, BOARD_HEIGHT * 16)
        self.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding
        )
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._grid: list[list[QColor | None]] = [
            [None for _ in range(BOARD_WIDTH)] for _ in range(BOARD_HEIGHT)
        ]
        self._current_piece: Tetromino | None = None
        self._score: int = 0
        self._lines: int = 0
        self._level: int = 1
        self._is_paused: bool = False
        self._is_game_over: bool = False

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

        self._spawn_piece()

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start (or restart) the game."""

        self._grid = [[None for _ in range(BOARD_WIDTH)] for _ in range(BOARD_HEIGHT)]
        self._score = 0
        self._lines = 0
        self._level = 1
        self._is_paused = False
        self._is_game_over = False
        self._spawn_piece()
        self._timer.start(self._drop_interval())
        self.update()

    def pause(self) -> None:
        """Toggle the paused state."""

        if self._is_game_over:
            return

        self._is_paused = not self._is_paused
        if self._is_paused:
            self._timer.stop()
        else:
            self._timer.start(self._drop_interval())
        self.update()

    def get_score(self) -> int:
        return self._score

    def get_lines(self) -> int:
        return self._lines

    def get_level(self) -> int:
        return self._level

    def is_paused(self) -> bool:
        return self._is_paused

    def is_game_over(self) -> bool:
        return self._is_game_over

    # ------------------------------------------------------------------
    # game logic
    # ------------------------------------------------------------------

    def _drop_interval(self) -> int:
        """Return the drop interval in milliseconds for the current level."""

        return max(80, 500 - (self._level - 1) * 40)

    def _spawn_piece(self) -> None:
        shape = random.choice(list(TETROMINOES.keys()))
        self._current_piece = Tetromino(shape)
        if self._collides(self._current_piece.absolute_cells()):
            self._is_game_over = True
            self._timer.stop()
            self.game_over_signal.emit()

    def _collides(self, cells: list[tuple[int, int]]) -> bool:
        for row, col in cells:
            if col < 0 or col >= BOARD_WIDTH or row >= BOARD_HEIGHT:
                return True
            if row >= 0 and self._grid[row][col] is not None:
                return True
        return False

    def _lock_piece(self) -> None:
        assert self._current_piece is not None
        for row, col in self._current_piece.absolute_cells():
            if 0 <= row < BOARD_HEIGHT and 0 <= col < BOARD_WIDTH:
                self._grid[row][col] = self._current_piece.color

        self._clear_lines()
        self._spawn_piece()

    def _clear_lines(self) -> None:
        new_grid: list[list[QColor | None]] = []
        cleared = 0

        for row in self._grid:
            if all(cell is not None for cell in row):
                cleared += 1
            else:
                new_grid.append(row)

        # prepend empty rows for each cleared line
        for _ in range(cleared):
            new_grid.insert(0, [None for _ in range(BOARD_WIDTH)])

        self._grid = new_grid

        if cleared > 0:
            points = [0, 100, 300, 500, 800][cleared] * self._level
            self._score += points
            self._lines += cleared
            self._level = 1 + self._lines // 10
            self._timer.setInterval(self._drop_interval())
            self.lines_cleared_signal.emit(cleared)

    def _tick(self) -> None:
        if self._current_piece is None or self._is_paused or self._is_game_over:
            return

        self._current_piece.row += 1
        if self._collides(self._current_piece.absolute_cells()):
            self._current_piece.row -= 1
            self._lock_piece()
        self.update()

    # ------------------------------------------------------------------
    # piece movement (called from key events)
    # ------------------------------------------------------------------

    def move_left(self) -> None:
        self._try_move(0, -1)

    def move_right(self) -> None:
        self._try_move(0, 1)

    def soft_drop(self) -> None:
        self._try_move(1, 0)
        self._score += 1
        self.update()

    def hard_drop(self) -> None:
        if self._current_piece is None or self._is_paused or self._is_game_over:
            return

        drop_distance = 0
        while True:
            self._current_piece.row += 1
            if self._collides(self._current_piece.absolute_cells()):
                self._current_piece.row -= 1
                break
            drop_distance += 1

        self._score += drop_distance * 2
        self._lock_piece()
        self.update()

    def rotate(self) -> None:
        if self._current_piece is None or self._is_paused or self._is_game_over:
            return

        old_cells = self._current_piece.cells
        self._current_piece.cells = self._current_piece.rotated_cells()

        # try simple wall kicks
        kicks = [0, -1, 1, -2, 2]
        kicked = False
        for kick in kicks:
            self._current_piece.col += kick
            if not self._collides(self._current_piece.absolute_cells()):
                kicked = True
                break
            self._current_piece.col -= kick

        if not kicked:
            self._current_piece.cells = old_cells
        self.update()

    def _try_move(self, d_row: int, d_col: int) -> None:
        if self._current_piece is None or self._is_paused or self._is_game_over:
            return

        self._current_piece.row += d_row
        self._current_piece.col += d_col

        if self._collides(self._current_piece.absolute_cells()):
            self._current_piece.row -= d_row
            self._current_piece.col -= d_col
            if d_row > 0:
                self._lock_piece()
        self.update()

    # ------------------------------------------------------------------
    # painting
    # ------------------------------------------------------------------

    def paintEvent(self, _event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # background
        painter.fillRect(self.rect(), QColor(20, 20, 30))

        # compute square cell size that fits the widget, centered
        cell = min(self.width() / BOARD_WIDTH, self.height() / BOARD_HEIGHT)
        offset_x = (self.width() - cell * BOARD_WIDTH) / 2
        offset_y = (self.height() - cell * BOARD_HEIGHT) / 2

        painter.save()
        painter.translate(offset_x, offset_y)

        # grid lines
        painter.setPen(QColor(40, 40, 50))
        for col in range(BOARD_WIDTH + 1):
            x = col * cell
            painter.drawLine(int(x), 0, int(x), int(cell * BOARD_HEIGHT))
        for row in range(BOARD_HEIGHT + 1):
            y = row * cell
            painter.drawLine(0, int(y), int(cell * BOARD_WIDTH), int(y))

        # locked blocks
        for row in range(BOARD_HEIGHT):
            for col in range(BOARD_WIDTH):
                color = self._grid[row][col]
                if color is not None:
                    self._draw_block(painter, row, col, color, cell)

        # current piece
        if self._current_piece is not None and not self._is_game_over:
            for row, col in self._current_piece.absolute_cells():
                if row >= 0:
                    self._draw_block(painter, row, col, self._current_piece.color, cell)

        painter.restore()

        # overlay text
        if self._is_paused or self._is_game_over:
            painter.fillRect(self.rect(), QColor(0, 0, 0, 150))
            painter.setPen(QColor(255, 255, 255))
            font = QFont("Arial", 24, QFont.Weight.Bold)
            painter.setFont(font)

            if self._is_game_over:
                text = "GAME OVER"
                sub_text = "Press R to restart or Esc to close"
            else:
                text = "PAUSED"
                sub_text = "Press P to resume"

            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, text)

            sub_font = QFont("Arial", 12)
            painter.setFont(sub_font)
            rect = self.rect()
            rect.translate(0, 40)
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, sub_text)

        painter.end()

    def _draw_block(
        self, painter: QPainter, row: int, col: int, color: QColor, cell: float
    ) -> None:
        """Draw a single block with a beveled edge."""

        x = col * cell
        y = row * cell

        # main fill
        painter.fillRect(int(x + 1), int(y + 1), int(cell - 2), int(cell - 2), color)

        # highlight (top-left)
        lighter = color.lighter(150)
        painter.setPen(lighter)
        painter.drawLine(int(x + 1), int(y + 1), int(x + cell - 2), int(y + 1))
        painter.drawLine(int(x + 1), int(y + 1), int(x + 1), int(y + cell - 2))

        # shadow (bottom-right)
        darker = color.darker(150)
        painter.setPen(darker)
        painter.drawLine(int(x + 1), int(y + cell - 2), int(x + cell - 2), int(y + cell - 2))
        painter.drawLine(int(x + cell - 2), int(y + 1), int(x + cell - 2), int(y + cell - 2))


class TetrisDialog(QDialog):
    """A dialog window containing a playable Tetris game."""

    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Tetris on the Phallic Ice Cream")
        self.setModal(False)
        self.resize(420, 720)
        self.setMinimumSize(260, 480)

        self._audio = TetrisAudio()

        self._board = TetrisBoard(self)
        self._board.lines_cleared_signal.connect(self._on_lines_cleared)
        self._board.game_over_signal.connect(self._on_game_over)

        # --- info panel ---
        self._score_label = QLabel("Score: 0")
        self._lines_label = QLabel("Lines: 0")
        self._level_label = QLabel("Level: 1")

        info_font = QFont("Arial", 14, QFont.Weight.Bold)
        for label in (self._score_label, self._lines_label, self._level_label):
            label.setFont(info_font)

        self._restart_button = QPushButton("Restart")
        self._restart_button.clicked.connect(self._restart)

        self._pause_button = QPushButton("Pause")
        self._pause_button.clicked.connect(self._toggle_pause)

        self._mute_button = QPushButton("Mute")
        self._mute_button.setCheckable(True)
        self._mute_button.toggled.connect(self._on_mute_toggled)

        info_layout = QVBoxLayout()
        info_layout.addWidget(self._score_label)
        info_layout.addWidget(self._lines_label)
        info_layout.addWidget(self._level_label)
        info_layout.addStretch()
        info_layout.addWidget(self._pause_button)
        info_layout.addWidget(self._mute_button)
        info_layout.addWidget(self._restart_button)

        # --- main layout ---
        main_layout = QHBoxLayout(self)
        main_layout.addWidget(self._board)
        main_layout.addLayout(info_layout)

        # NOTE: do NOT start the board here. The game timer must only run
        # while the dialog is visible — otherwise pieces keep falling in the
        # background and the game-over SFX eventually plays audibly even when
        # nobody is playing. See showEvent / hideEvent.
        self._board.setFocus()

        self._music_started: bool = False
        self._game_started: bool = False

    # ------------------------------------------------------------------
    # show / hide — keep audio + game tied to visibility
    # ------------------------------------------------------------------

    def showEvent(self, event: QShowEvent) -> None:
        """Start the game and music the first time the dialog becomes visible."""

        super().showEvent(event)
        if not self._game_started:
            self._board.start()
            self._game_started = True
        if not self._music_started and not self._board.is_game_over():
            self._audio.start_music()
            self._music_started = True
        self._board.setFocus()

    def hideEvent(self, event: QHideEvent) -> None:
        """Stop music and pause the game when the dialog is hidden."""

        self._audio.stop_music()
        if not self._board.is_game_over() and not self._board.is_paused():
            self._board.pause()
            self._pause_button.setText("Resume")
        super().hideEvent(event)

    # ------------------------------------------------------------------
    # key handling
    # ------------------------------------------------------------------

    def keyPressEvent(self, event: QKeyEvent) -> None:
        """
        Handle keyboard input for Tetris gameplay.

        Parameters
        ----------
        event
            The key event to handle.
        """

        key = event.key()

        if key == Qt.Key.Key_Escape:
            self.close()
            return

        if key == Qt.Key.Key_P:
            self._toggle_pause()
            return

        if key == Qt.Key.Key_R:
            self._restart()
            return

        if self._board.is_paused() or self._board.is_game_over():
            return

        if key == Qt.Key.Key_Left:
            self._board.move_left()
        elif key == Qt.Key.Key_Right:
            self._board.move_right()
        elif key == Qt.Key.Key_Down:
            self._board.soft_drop()
        elif key == Qt.Key.Key_Up:
            self._board.rotate()
        elif key == Qt.Key.Key_Space:
            self._board.hard_drop()
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _update_labels(self, _lines_cleared: int) -> None:
        self._score_label.setText(f"Score: {self._board.get_score()}")
        self._lines_label.setText(f"Lines: {self._board.get_lines()}")
        self._level_label.setText(f"Level: {self._board.get_level()}")

    def _on_lines_cleared(self, lines_cleared: int) -> None:
        """Update labels and play the line-clear SFX."""

        self._update_labels(lines_cleared)
        self._audio.play_line_clear()

    def _on_game_over(self) -> None:
        """Update labels, stop music, and play the game-over SFX."""

        self._update_labels(0)
        self._audio.stop_music()
        self._audio.play_game_over()

    def _restart(self) -> None:
        self._board.start()
        self._update_labels(0)
        self._pause_button.setText("Pause")
        self._game_started = True
        self._music_started = True
        self._audio.start_music()
        self._board.setFocus()

    def _toggle_pause(self) -> None:
        self._board.pause()
        paused = self._board.is_paused()
        self._pause_button.setText("Resume" if paused else "Pause")
        if paused:
            self._audio.pause_music()
        else:
            self._audio.resume_music()
        self._board.setFocus()

    def _on_mute_toggled(self, muted: bool) -> None:
        """Toggle all audio on/off."""

        self._mute_button.setText("Unmute" if muted else "Mute")
        self._audio.set_music_enabled(not muted)
        self._audio.set_sfx_enabled(not muted)
        if not muted and not self._board.is_game_over() and self._game_started and self._music_started:
            self._audio.start_music()
        self._board.setFocus()

    def closeEvent(self, event: QCloseEvent) -> None:
        """Stop audio when the dialog closes."""

        self._audio.stop_music()
        self._music_started = False
        super().closeEvent(event)
