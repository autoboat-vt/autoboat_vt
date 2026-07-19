"""Audio playback for the Tetris easter egg."""

from __future__ import annotations

import random
from pathlib import Path

from qtpy.QtCore import QUrl
from qtpy.QtMultimedia import QAudioOutput, QMediaPlayer, QSoundEffect
try:
    from qtpy.QtMultimedia import QMediaContent
except ImportError:
    QMediaContent = None

from utils import constants

__all__ = ["TetrisAudio"]


class TetrisAudio:
    """
    Manage music and sound effects for Tetris.

    Uses ``QMediaPlayer`` for looping background music (so we get gapless
    playback and volume control) and ``QSoundEffect`` for short one-shot SFX
    (low latency, supports overlapping plays).

    All file paths are resolved from ``constants.TETRIS_MUSIC_DIR`` and
    ``constants.TETRIS_SFX_DIR``. If the files don't exist (e.g. assets not
    shipped), every method silently no-ops so the game still works without
    audio.
    """

    def __init__(self) -> None:
        self._music_player: QMediaPlayer | None = None
        self._music_output: QAudioOutput | None = None
        self._music_tracks: list[Path] = []
        self._current_track_index: int = 0
        self._music_enabled: bool = True
        self._sfx_enabled: bool = True

        self._sfx_line_clear: QSoundEffect | None = None
        self._sfx_game_over: QSoundEffect | None = None

        self._load_tracks()
        self._load_sfx()

    # ------------------------------------------------------------------
    # setup
    # ------------------------------------------------------------------

    def _load_tracks(self) -> None:
        """Discover music tracks in the tetris music directory."""

        music_dir = constants.TETRIS_MUSIC_DIR
        if not music_dir.is_dir():
            return

        self._music_tracks = sorted(
            p for p in music_dir.iterdir() if p.is_file() and p.suffix.lower() in {".mp3", ".wav", ".ogg"}
        )

        if self._music_tracks:
            self._music_player = QMediaPlayer()
            if hasattr(self._music_player, "setAudioOutput"):
                self._music_output = QAudioOutput()
                self._music_player.setAudioOutput(self._music_output)
                self._music_output.setVolume(0.5)
            else:
                self._music_output = None
                self._music_player.setVolume(50)
            # advance to the next track when the current one ends
            self._music_player.mediaStatusChanged.connect(self._on_media_status_changed)

    def _load_sfx(self) -> None:
        """Pre-load the sound effects so playback is instant."""

        self._sfx_line_clear = self._make_sound_effect(constants.TETRIS_SFX_DIR / "line_break.wav")
        self._sfx_game_over = self._make_sound_effect(constants.TETRIS_SFX_DIR / "game_over.wav")

    @staticmethod
    def _make_sound_effect(path: Path) -> QSoundEffect | None:
        """
        Create a pre-loaded ``QSoundEffect`` for ``path``, or ``None``.

        Parameters
        ----------
        path
            The audio file to load.
        """

        if not path.is_file():
            return None

        effect = QSoundEffect()
        effect.setSource(QUrl.fromLocalFile(path.as_posix()))
        effect.setVolume(0.7)
        return effect

    # ------------------------------------------------------------------
    # music
    # ------------------------------------------------------------------

    def start_music(self) -> None:
        """Begin playing background music, picking a random starting track."""

        if not self._music_enabled or self._music_player is None or not self._music_tracks:
            return

        self._current_track_index = random.randrange(len(self._music_tracks))
        self._play_current_track()

    def stop_music(self) -> None:
        """Stop the background music."""

        if self._music_player is not None:
            self._music_player.stop()

    def pause_music(self) -> None:
        """Pause the background music."""

        if self._music_player is not None:
            self._music_player.pause()

    def resume_music(self) -> None:
        """Resume the background music after a pause."""

        if self._music_enabled and self._music_player is not None:
            self._music_player.play()

    def set_music_enabled(self, enabled: bool) -> None:
        """
        Enable or disable background music.

        Parameters
        ----------
        enabled
            ``True`` to allow music, ``False`` to stop and suppress it.
        """

        self._music_enabled = enabled
        if not enabled:
            self.stop_music()
        else:
            self.start_music()

    def set_music_volume(self, volume: float) -> None:
        """
        Set the music volume.

        Parameters
        ----------
        volume
            A float from 0.0 (silent) to 1.0 (full volume).
        """

        if self._music_output is not None:
            self._music_output.setVolume(max(0.0, min(1.0, volume)))
        elif self._music_player is not None:
            self._music_player.setVolume(int(max(0.0, min(1.0, volume)) * 100))

    def _play_current_track(self) -> None:
        """Load and play the track at the current index."""

        if self._music_player is None or not self._music_tracks:
            return

        track = self._music_tracks[self._current_track_index]
        url = QUrl.fromLocalFile(track.as_posix())
        if hasattr(self._music_player, "setSource"):
            self._music_player.setSource(url)
        else:
            if QMediaContent is not None:
                self._music_player.setMedia(QMediaContent(url))
        self._music_player.play()

    def _on_media_status_changed(self, status: QMediaPlayer.MediaStatus) -> None:
        """
        Advance to the next track when the current one finishes.

        Parameters
        ----------
        status
            The new media status from the player.
        """

        if status == QMediaPlayer.MediaStatus.EndOfMedia and self._music_tracks:
            self._current_track_index = (self._current_track_index + 1) % len(self._music_tracks)
            self._play_current_track()

    # ------------------------------------------------------------------
    # sound effects
    # ------------------------------------------------------------------

    def play_line_clear(self) -> None:
        """Play the line-clear sound effect."""

        if self._sfx_enabled and self._sfx_line_clear is not None:
            self._sfx_line_clear.play()

    def play_game_over(self) -> None:
        """Play the game-over sound effect."""

        if self._sfx_enabled and self._sfx_game_over is not None:
            self._sfx_game_over.play()

    def set_sfx_enabled(self, enabled: bool) -> None:
        """
        Enable or disable sound effects.

        Parameters
        ----------
        enabled
            ``True`` to play SFX, ``False`` to suppress them.
        """

        self._sfx_enabled = enabled

    # ------------------------------------------------------------------
    # cleanup
    # ------------------------------------------------------------------

    def cleanup(self) -> None:
        """Stop all playback and release resources."""

        self.stop_music()
        for sfx in (self._sfx_line_clear, self._sfx_game_over):
            if sfx is not None:
                sfx.stop()
