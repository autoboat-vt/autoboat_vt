"""A playable Snake game easter egg."""

from __future__ import annotations

import random
from collections import deque

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

from .snake_audio import SnakeAudio

__all__ = ["SnakeDialog"]

# Board dimensions (in blocks).
COLS: int = 25
ROWS: int = 25
BLOCK_SIZE: int = 22  # pixels per block

# Tick interval (ms) — lower is faster.
TICK_MS: int = 150


class SnakeBoard(QFrame):
    """The playable Snake board widget."""

    score_changed_signal = Signal(int)
    food_eaten_signal = Signal()
    game_over_signal = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.setMinimumSize(COLS * 12, ROWS * 12)
        self.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding
        )
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._snake: deque[tuple[int, int]] = deque()
        self._direction: tuple[int, int] = (1, 0)
        self._next_direction: tuple[int, int] = (1, 0)
        self._food: tuple[int, int] = (0, 0)
        self._score: int = 0
        self._is_paused: bool = False
        self._is_game_over: bool = False

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        
        self._spawn_food()

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start (or restart) the game."""

        mid_row = ROWS // 2
        mid_col = COLS // 2
        self._snake = deque(
            [(mid_row, mid_col), (mid_row, mid_col - 1), (mid_row, mid_col - 2)]
        )
        self._direction = (1, 0)
        self._next_direction = (1, 0)
        self._score = 0
        self._is_paused = False
        self._is_game_over = False
        self._spawn_food()
        self._timer.start(TICK_MS)
        self.score_changed_signal.emit(self._score)
        self.update()

    def pause(self) -> None:
        """Toggle the paused state."""

        if self._is_game_over:
            return

        self._is_paused = not self._is_paused
        if self._is_paused:
            self._timer.stop()
        else:
            self._timer.start(TICK_MS)
        self.update()

    def set_direction(self, d_row: int, d_col: int) -> None:
        """Queue a new direction (ignored if it would reverse the snake)."""

        # forbid reversing directly into yourself
        if (d_row, d_col) == (-self._direction[0], -self._direction[1]):
            return

        self._next_direction = (d_row, d_col)

    def get_score(self) -> int:
        return self._score

    def is_paused(self) -> bool:
        return self._is_paused

    def is_game_over(self) -> bool:
        return self._is_game_over

    # ------------------------------------------------------------------
    # game logic
    # ------------------------------------------------------------------

    def _spawn_food(self) -> None:
        """Place food on a random empty cell."""

        occupied = set(self._snake)
        free = [
            (r, c)
            for r in range(ROWS)
            for c in range(COLS)
            if (r, c) not in occupied
        ]
        if not free:
            # board full — you win (treat as game over)
            self._is_game_over = True
            self._timer.stop()
            self.game_over_signal.emit()
            return
        self._food = random.choice(free)

    def _tick(self) -> None:
        if self._is_paused or self._is_game_over:
            return

        self._direction = self._next_direction
        head_row, head_col = self._snake[0]
        new_head = (head_row + self._direction[0], head_col + self._direction[1])

        # wall collision
        if not (0 <= new_head[0] < ROWS and 0 <= new_head[1] < COLS):
            self._game_over()
            return

        # self collision (check against body excluding the tail, which will move)
        body = list(self._snake)[:-1]
        if new_head in body:
            self._game_over()
            return

        self._snake.appendleft(new_head)

        if new_head == self._food:
            self._score += 10
            self.score_changed_signal.emit(self._score)
            self.food_eaten_signal.emit()
            self._spawn_food()
        else:
            self._snake.pop()

        self.update()

    def _game_over(self) -> None:
        self._is_game_over = True
        self._timer.stop()
        self.game_over_signal.emit()
        self.update()

    # ------------------------------------------------------------------
    # painting
    # ------------------------------------------------------------------

    def paintEvent(self, _event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # background
        painter.fillRect(self.rect(), QColor(15, 30, 15))

        # compute square cell size that fits the widget, centered
        cell = min(self.width() / COLS, self.height() / ROWS)
        offset_x = (self.width() - cell * COLS) / 2
        offset_y = (self.height() - cell * ROWS) / 2

        painter.save()
        painter.translate(offset_x, offset_y)

        # grid (subtle)
        painter.setPen(QColor(25, 45, 25))
        for col in range(COLS + 1):
            x = col * cell
            painter.drawLine(int(x), 0, int(x), int(cell * ROWS))
        for row in range(ROWS + 1):
            y = row * cell
            painter.drawLine(0, int(y), int(cell * COLS), int(y))

        # food
        fr, fc = self._food
        self._draw_block(painter, fr, fc, QColor(240, 40, 40), cell)

        # snake
        for i, (r, c) in enumerate(self._snake):
            # head is brighter
            color = QColor(60, 230, 60) if i == 0 else QColor(40, 180, 40)
            self._draw_block(painter, r, c, color, cell)

        painter.restore()

        # overlay
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

        painter.fillRect(int(x + 1), int(y + 1), int(cell - 2), int(cell - 2), color)

        lighter = color.lighter(150)
        painter.setPen(lighter)
        painter.drawLine(int(x + 1), int(y + 1), int(x + cell - 2), int(y + 1))
        painter.drawLine(int(x + 1), int(y + 1), int(x + 1), int(y + cell - 2))

        darker = color.darker(150)
        painter.setPen(darker)
        painter.drawLine(int(x + 1), int(y + cell - 2), int(x + cell - 2), int(y + cell - 2))
        painter.drawLine(int(x + cell - 2), int(y + 1), int(x + cell - 2), int(y + cell - 2))


class SnakeDialog(QDialog):
    """A dialog window containing a playable Snake game."""

    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Snake")
        self.setModal(False)
        self.resize(620, 600)
        self.setMinimumSize(420, 380)

        self._audio = SnakeAudio()

        self._board = SnakeBoard(self)
        self._board.score_changed_signal.connect(self._on_score_changed)
        self._board.food_eaten_signal.connect(self._audio.play_eat_food)
        self._board.game_over_signal.connect(self._on_game_over)

        # --- info panel ---
        self._score_label = QLabel("Score: 0")
        self._high_label = QLabel("Best: 0")
        self._high_score: int = 0

        info_font = QFont("Arial", 14, QFont.Weight.Bold)
        self._score_label.setFont(info_font)
        self._high_label.setFont(info_font)

        self._restart_button = QPushButton("Restart")
        self._restart_button.clicked.connect(self._restart)

        self._pause_button = QPushButton("Pause")
        self._pause_button.clicked.connect(self._toggle_pause)

        self._mute_button = QPushButton("Mute")
        self._mute_button.setCheckable(True)
        self._mute_button.toggled.connect(self._on_mute_toggled)

        info_layout = QVBoxLayout()
        info_layout.addWidget(self._score_label)
        info_layout.addWidget(self._high_label)
        info_layout.addStretch()
        info_layout.addWidget(self._pause_button)
        info_layout.addWidget(self._mute_button)
        info_layout.addWidget(self._restart_button)

        # --- main layout ---
        main_layout = QHBoxLayout(self)
        main_layout.addWidget(self._board)
        main_layout.addLayout(info_layout)

        self._music_started: bool = False
        self._game_started: bool = False

    # ------------------------------------------------------------------
    # show / hide
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
        """Pause the game and music when the dialog is hidden."""

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
        Handle keyboard input for Snake gameplay.

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

        if key in (Qt.Key.Key_Left, Qt.Key.Key_A):
            self._board.set_direction(0, -1)
        elif key in (Qt.Key.Key_Right, Qt.Key.Key_D):
            self._board.set_direction(0, 1)
        elif key in (Qt.Key.Key_Up, Qt.Key.Key_W):
            self._board.set_direction(-1, 0)
        elif key in (Qt.Key.Key_Down, Qt.Key.Key_S):
            self._board.set_direction(1, 0)
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _on_score_changed(self, score: int) -> None:
        self._score_label.setText(f"Score: {score}")
        if score > self._high_score:
            self._high_score = score
            self._high_label.setText(f"Best: {self._high_score}")

    def _on_game_over(self) -> None:
        self._on_score_changed(self._board.get_score())
        self._audio.stop_music()
        self._audio.play_game_over()

    def _restart(self) -> None:
        self._board.start()
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
        """Clean up audio when the dialog closes."""

        self._audio.cleanup()
        super().closeEvent(event)
