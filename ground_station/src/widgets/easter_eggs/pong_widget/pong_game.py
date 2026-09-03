"""A playable Pong game easter egg."""

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
    QPen,
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

from .pong_audio import PongAudio

__all__ = ["PongDialog"]

# Board dimensions in pixels.
WIDTH: int = 640
HEIGHT: int = 400

PADDLE_W: int = 12
PADDLE_H: int = 80
PADDLE_SPEED: int = 6

BALL_SIZE: int = 12
BALL_SPEED: int = 5

WIN_SCORE: int = 7


class PongBoard(QFrame):
    """The playable Pong board widget."""

    score_changed_signal = Signal(int, int)  # left, right
    collision_signal = Signal()
    game_over_signal = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.setMinimumSize(320, 200)
        self.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding
        )
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._left_y: int = HEIGHT // 2 - PADDLE_H // 2
        self._right_y: int = HEIGHT // 2 - PADDLE_H // 2
        self._ball_x: float = WIDTH / 2
        self._ball_y: float = HEIGHT / 2
        self._ball_vx: float = BALL_SPEED
        self._ball_vy: float = 0.0
        self._left_score: int = 0
        self._right_score: int = 0
        self._is_paused: bool = False
        self._is_game_over: bool = False
        self._left_up: bool = False
        self._left_down: bool = False
        self._pending_direction: int = -1  # set by _begin_countdown

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

        # countdown before each serve (3, 2, 1, then GO)
        self._countdown: int = 0  # 0 means "no countdown active"
        self._countdown_timer = QTimer(self)
        self._countdown_timer.setSingleShot(True)
        self._countdown_timer.timeout.connect(self._on_countdown_tick)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start (or restart) the game."""

        self._left_y = HEIGHT // 2 - PADDLE_H // 2
        self._right_y = HEIGHT // 2 - PADDLE_H // 2
        self._left_score = 0
        self._right_score = 0
        self._is_paused = False
        self._is_game_over = False
        self._timer.start(16)  # roughly 60 FPS
        self.score_changed_signal.emit(self._left_score, self._right_score)
        self._begin_countdown(direction=-1)
        self.update()

    def pause(self) -> None:
        """Toggle the paused state."""

        if self._is_game_over:
            return

        self._is_paused = not self._is_paused
        if self._is_paused:
            self._timer.stop()
            self._countdown_timer.stop()
        else:
            self._timer.start(16)
            # resume the countdown if it was mid-count
            if self._countdown > 0:
                self._countdown_timer.start(1000)
        self.update()

    def set_left_up(self, pressed: bool) -> None:
        self._left_up = pressed

    def set_left_down(self, pressed: bool) -> None:
        self._left_down = pressed

    def get_scores(self) -> tuple[int, int]:
        return self._left_score, self._right_score

    def is_paused(self) -> bool:
        return self._is_paused

    def is_game_over(self) -> bool:
        return self._is_game_over

    # ------------------------------------------------------------------
    # game logic
    # ------------------------------------------------------------------

    def _begin_countdown(self, direction: int) -> None:
        """Center the ball and run a 3-2-1 countdown before serving."""

        self._pending_direction = direction
        self._ball_x = WIDTH / 2
        self._ball_y = HEIGHT / 2
        self._ball_vx = 0.0
        self._ball_vy = 0.0
        self._countdown = 3
        self._countdown_timer.start(1000)  # 1 second per tick
        self.update()

    def _on_countdown_tick(self) -> None:
        """Advance the countdown; when it hits zero, launch the ball."""

        if self._is_paused or self._is_game_over:
            return

        self._countdown -= 1
        if self._countdown <= 0:
            self._countdown = 0
            self._reset_ball(direction=self._pending_direction)
        else:
            self._countdown_timer.start(1000)
        self.update()

    def _reset_ball(self, direction: int) -> None:
        """Reset the ball to center, moving toward ``direction`` (-1 or 1)."""

        self._ball_x = WIDTH / 2
        self._ball_y = HEIGHT / 2
        
        angle = random.uniform(-0.4, 0.4)
        self._ball_vx = direction * BALL_SPEED
        self._ball_vy = angle * BALL_SPEED

    def _tick(self) -> None:
        if self._is_paused or self._is_game_over:
            return

        # --- paddles ---
        if self._left_up:
            self._left_y = max(0, self._left_y - PADDLE_SPEED)
        if self._left_down:
            self._left_y = min(HEIGHT - PADDLE_H, self._left_y + PADDLE_SPEED)

        # right paddle is always CPU-controlled
        self._move_ai()

        # --- ball ---
        # skip ball physics during the pre-serve countdown
        if self._countdown > 0:
            self.update()
            return

        self._ball_x += self._ball_vx
        self._ball_y += self._ball_vy

        # top/bottom wall bounce
        if self._ball_y <= 0:
            self._ball_y = 0
            self._ball_vy = abs(self._ball_vy)
            self.collision_signal.emit()
        elif self._ball_y + BALL_SIZE >= HEIGHT:
            self._ball_y = HEIGHT - BALL_SIZE
            self._ball_vy = -abs(self._ball_vy)
            self.collision_signal.emit()

        # left paddle collision
        if (
            self._ball_x <= PADDLE_W + 4
            and self._left_y <= self._ball_y + BALL_SIZE <= self._left_y + PADDLE_H
            and self._ball_vx < 0
        ):
            self._ball_vx = abs(self._ball_vx)
            # add angle based on where it hit the paddle
            offset = (self._ball_y + BALL_SIZE / 2) - (self._left_y + PADDLE_H / 2)
            self._ball_vy = offset * 0.15
            self.collision_signal.emit()

        # right paddle collision
        if (
            self._ball_x + BALL_SIZE >= WIDTH - PADDLE_W - 4
            and self._right_y <= self._ball_y + BALL_SIZE <= self._right_y + PADDLE_H
            and self._ball_vx > 0
        ):
            self._ball_vx = -abs(self._ball_vx)
            offset = (self._ball_y + BALL_SIZE / 2) - (self._right_y + PADDLE_H / 2)
            self._ball_vy = offset * 0.15
            self.collision_signal.emit()

        # scoring
        if self._ball_x < 0:
            self._right_score += 1
            self.score_changed_signal.emit(self._left_score, self._right_score)
            self._check_win_or_reset(direction=1)
        elif self._ball_x > WIDTH:
            self._left_score += 1
            self.score_changed_signal.emit(self._left_score, self._right_score)
            self._check_win_or_reset(direction=-1)

        self.update()

    def _move_ai(self) -> None:
        """Simple AI that tracks the ball with a slight delay."""

        center = self._right_y + PADDLE_H / 2
        target = self._ball_y + BALL_SIZE / 2
        # only react when ball is moving toward the AI
        if self._ball_vx > 0:
            if target < center - 4:
                self._right_y = max(0, self._right_y - PADDLE_SPEED)
            elif target > center + 4:
                self._right_y = min(HEIGHT - PADDLE_H, self._right_y + PADDLE_SPEED)

    def _check_win_or_reset(self, direction: int) -> None:
        if self._left_score >= WIN_SCORE or self._right_score >= WIN_SCORE:
            self._is_game_over = True
            self._timer.stop()
            self._countdown_timer.stop()
            self._countdown = 0
            self.game_over_signal.emit()
        else:
            self._begin_countdown(direction=direction)

    # ------------------------------------------------------------------
    # painting
    # ------------------------------------------------------------------

    def paintEvent(self, _event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # scale the logical WIDTH * HEIGHT playfield to fit the widget,
        # preserving aspect ratio and centering
        scale = min(self.width() / WIDTH, self.height() / HEIGHT)
        offset_x = (self.width() - WIDTH * scale) / 2
        offset_y = (self.height() - HEIGHT * scale) / 2

        # background (fills the whole widget)
        painter.fillRect(self.rect(), QColor(15, 15, 25))

        # center dashed line — drawn in widget coordinates so its
        # thickness and dash size scale with the window
        center_x = int(offset_x + (WIDTH * scale) / 2)
        play_top = offset_y
        play_h = HEIGHT * scale
        dash = max(6.0, 16.0 * scale)
        gap = dash
        line_pen = QPen(QColor(80, 80, 100))
        line_pen.setWidthF(max(1.0, 2.0 * scale))
        painter.setPen(line_pen)
        cy = play_top
        while cy < play_top + play_h:
            cy_end = min(cy + dash, play_top + play_h)
            painter.drawLine(center_x, int(cy), center_x, int(cy_end))
            cy += dash + gap

        painter.save()
        painter.translate(offset_x, offset_y)
        painter.scale(scale, scale)

        # paddles
        painter.setBrush(QColor(240, 240, 240))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(4, self._left_y, PADDLE_W, PADDLE_H)
        painter.drawRect(WIDTH - PADDLE_W - 4, self._right_y, PADDLE_W, PADDLE_H)

        # ball
        painter.setBrush(QColor(240, 240, 240))
        painter.drawEllipse(int(self._ball_x), int(self._ball_y), BALL_SIZE, BALL_SIZE)

        painter.restore()

        # overlay (drawn in widget coordinates so text stays crisp)
        if self._is_paused or self._is_game_over:
            painter.fillRect(self.rect(), QColor(0, 0, 0, 150))
            painter.setPen(QColor(255, 255, 255))
            font = QFont("Arial", 24, QFont.Weight.Bold)
            painter.setFont(font)

            if self._is_game_over:
                winner = "You" if self._left_score > self._right_score else "CPU"
                text = f"{winner} WINS!"
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
        elif self._countdown > 0:
            # pre-serve countdown overlay
            painter.fillRect(self.rect(), QColor(0, 0, 0, 120))
            painter.setPen(QColor(255, 255, 255))
            countdown_font = QFont("Arial", 72, QFont.Weight.Bold)
            painter.setFont(countdown_font)
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, str(self._countdown))

        painter.end()


class PongDialog(QDialog):
    """A dialog window containing a playable Pong game."""

    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Pong")
        self.setModal(False)
        self.resize(820, 460)
        self.setMinimumSize(440, 280)

        self._audio = PongAudio()

        self._board = PongBoard(self)
        self._board.score_changed_signal.connect(self._on_score_changed)
        self._board.collision_signal.connect(self._audio.play_collision)
        self._board.game_over_signal.connect(self._on_game_over)

        # --- info panel ---
        self._score_label = QLabel("0 : 0")
        score_font = QFont("Arial", 20, QFont.Weight.Bold)
        self._score_label.setFont(score_font)
        self._score_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._controls_label = QLabel(
            "Move: W / S   (or ↑ / ↓)\n"
            "CPU controls the right paddle\n\n"
            "P: pause   R: restart   Esc: close"
        )
        self._controls_label.setWordWrap(True)

        self._restart_button = QPushButton("Restart")
        self._restart_button.clicked.connect(self._restart)

        self._pause_button = QPushButton("Pause")
        self._pause_button.clicked.connect(self._toggle_pause)

        self._mute_button = QPushButton("Mute")
        self._mute_button.setCheckable(True)
        self._mute_button.toggled.connect(self._on_mute_toggled)

        info_layout = QVBoxLayout()
        info_layout.addWidget(self._score_label)
        info_layout.addWidget(self._controls_label)
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
        Handle keyboard input for Pong gameplay.

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

        # player paddle: W / S (also accept Up / Down)
        if key in (Qt.Key.Key_W, Qt.Key.Key_Up):
            self._board.set_left_up(True)
        elif key in (Qt.Key.Key_S, Qt.Key.Key_Down):
            self._board.set_left_down(True)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        """Handle key releases so the paddle stops moving."""

        key = event.key()
        if key in (Qt.Key.Key_W, Qt.Key.Key_Up):
            self._board.set_left_up(False)
        elif key in (Qt.Key.Key_S, Qt.Key.Key_Down):
            self._board.set_left_down(False)
        else:
            super().keyReleaseEvent(event)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _on_score_changed(self, left: int, right: int) -> None:
        self._score_label.setText(f"{left} : {right}")

    def _on_game_over(self) -> None:
        left, right = self._board.get_scores()
        self._on_score_changed(left, right)
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
