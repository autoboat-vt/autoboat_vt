from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path

from qtpy.QtCore import QObject, Qt, Signal
from qtpy.QtGui import QKeyEvent, QKeySequence

from utils import constants

__all__ = ["KeybindManager", "get_keybind_manager", "normalize_key_string", "qt_key_event_to_string"]

# we do this to avoid a circular import between this module and `constants.py`
_KeybindManagerInstance: KeybindManager | None = None

def get_keybind_manager() -> KeybindManager:
    """
    Return the shared ```KeybindManager``` instance.

    The instance is created on first call. Subsequent calls return the same
    object so that widgets, shortcuts, and the frontend bridge all stay synced.

    Returns
    -------
    KeybindManager
        The shared keybind manager instance.
    """

    global _KeybindManagerInstance  # noqa: PLW0603 - intentional singleton, mirrors constants.SM
    if _KeybindManagerInstance is None:
        _KeybindManagerInstance = KeybindManager()

    return _KeybindManagerInstance


def normalize_key_string(raw: str) -> str:
    """
    Normalize a key combination string for storage and comparison.

    Parameters
    ----------
    raw
        The raw key combination string, e.g. ``"Ctrl+Shift+P"`` or ``"f"``.

    Returns
    -------
    str
        The normalized string with single spaces around ``+`` separators and
        trimmed whitespace. Modifiers are title-cased so that ``"ctrl+shift+p"``
        and ``"Ctrl+Shift+P"`` compare equal.
    """

    parts = [part.strip().title() for part in raw.split("+") if part.strip()]
    return "+".join(parts)


class KeybindManager(QObject):
    """
    Manage keybind registration, persistence, and dispatch.

    The manager reads the default bindings defined in
    ``app_data/git_keep/defaults_examples/keybinds/default.json`` (exposed via
    ``constants.KEYBINDS_DIR``) and keeps an in-memory copy that callers can
    update. Writes are persisted back to ``app_state.json``.

    Attributes
    ----------
    bindings_changed: ``Signal``
        Emitted whenever a binding is added, removed, or changed. Carries the
        full bindings dict so listeners can re-register shortcuts.

    Inherits
    -------
    ``QObject``
    """

    bindings_changed = Signal(dict)

    def __init__(self) -> None:
        super().__init__()

        self._handlers: dict[str, Callable[[], None]] = {}
        self._load_bindings()

    def _load_bindings(self) -> None:
        """
        Load the current bindings from the state file into memory.

        Any actions present in ``default.json`` but missing from the cached
        state (e.g. a newly added action shipped in an update) are merged in
        from the defaults so they become available without a manual reset.
        """

        stored = constants.SM.read_dict("keybindings")
        if not isinstance(stored, dict) or not stored:
            self._bindings: dict[str, dict[str, str]] = self._load_default_bindings()
            return

        self._bindings = {}
        for action, info in stored.items():
            entry = dict(info)
            
            if "key" in entry and isinstance(entry["key"], str):
                entry["key"] = normalize_key_string(entry["key"])
            
            self._bindings[action] = entry

        # merge in any actions that exist in the defaults but not in the cached state
        defaults = self._load_default_bindings()
        merged_any = False
        for action, info in defaults.items():
            if action not in self._bindings:
                self._bindings[action] = dict(info)
                merged_any = True

        if merged_any:
            self._persist()

    @staticmethod
    def _load_default_bindings() -> dict[str, dict[str, str]]:
        """
        Load the default keybindings from ``KEYBINDS_DIR/default.json``.

        Returns
        -------
        dict[str, dict[str, str]]
            A fresh copy of the default bindings, with each ``key`` normalized.
        """

        default_path = Path(constants.KEYBINDS_DIR / "default.json")
        with open(file=default_path, mode="r", encoding="utf-8") as f:
            raw = json.load(f)

        return {
            action: {**info, "key": normalize_key_string(info["key"])}
            for action, info in raw.items()
        }

    def get_bindings(self) -> dict[str, dict[str, str]]:
        """
        Return a shallow copy of the current bindings.

        Returns
        -------
        dict[str, dict[str, str]]
            A dict mapping action keys to binding info dicts.
        """

        return {action: dict(info) for action, info in self._bindings.items()}

    def get_binding(self, action: str) -> dict[str, str] | None:
        """
        Return the binding info for a single action, or ``None``.

        Parameters
        ----------
        action
            The action key, e.g. ``"focus_boat"``.

        Returns
        -------
        dict[str, str] | None
            The binding info dict, or ``None`` if the action is unknown.
        """

        info = self._bindings.get(action)
        return dict(info) if info is not None else None

    def get_key(self, action: str) -> str | None:
        """
        Return the normalized key string for an action, or ``None``.

        Parameters
        ----------
        action
            The action key.

        Returns
        -------
        str | None
            The normalized key string (e.g. ``"Ctrl+Shift+P"``) or ``None``.
        """

        info = self._bindings.get(action)
        if info is None:
            return None
        
        key = info.get("key")
        return normalize_key_string(key) if isinstance(key, str) else None

    def get_actions_by_scope(self, scope: str) -> dict[str, dict[str, str]]:
        """
        Return all bindings whose ``scope`` matches the given value.

        Parameters
        ----------
        scope
            Either ``"app"`` or ``"map"``.

        Returns
        -------
        dict[str, dict[str, str]]
            A dict of action key -> binding info for the matching scope.
        """

        return {
            action: dict(info)
            for action, info in self._bindings.items()
            if info.get("scope") == scope
        }

    def find_conflict(self, key: str, ignore_action: str | None = None) -> str | None:
        """
        Find an action (other than ``ignore_action``) already bound to ``key``.

        Parameters
        ----------
        key
            The normalized key string to check.
        ignore_action
            An action key to exclude from the conflict check (e.g. the action
            currently being rebound).

        Returns
        -------
        str | None
            The action key of the conflicting binding, or ``None`` if there is
            no conflict.
        """

        normalized = normalize_key_string(key)

        # empty keys (unbound) never conflict with anything
        if not normalized:
            return None

        for action, info in self._bindings.items():
            if action == ignore_action:
                continue
            
            existing = info.get("key")
            if isinstance(existing, str) and existing and normalize_key_string(existing) == normalized:
                return action
        
        return None

    def set_key(self, action: str, key: str) -> str | None:
        """
        Set the key for an action, persist it, and emit ``bindings_changed``.

        Parameters
        ----------
        action
            The action key to update.
        key
            The new key combination string (will be normalized).

        Returns
        -------
        str | None
            The action key of a conflicting binding if one exists (the update
            is still applied — the caller may choose to warn the user), or
            ``None`` on a clean update. Returns ``None`` if the action is
            unknown (nothing was changed).
        """

        info = self._bindings.get(action)
        if info is None:
            return None

        normalized = normalize_key_string(key)
        # an empty key means "unbound" — no conflict is possible, skip the check
        conflict = self.find_conflict(normalized, ignore_action=action) if normalized else None

        info["key"] = normalized
        self._persist()

        self.bindings_changed.emit(self.get_bindings())
        return conflict

    def reset_to_defaults(self) -> None:
        """
        Reset all bindings to the defaults defined in ``KEYBINDS_DIR/default.json``.

        Emits ``bindings_changed`` after persisting.
        """

        self._bindings = self._load_default_bindings()
        self._persist()
        self.bindings_changed.emit(self.get_bindings())

    def save_to_file(self, file_path: str | Path) -> None:
        """
        Write the current bindings to a JSON file.

        Parameters
        ----------
        file_path
            The path to write to. Parent directories are created if needed.
        """

        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(path, mode="w", encoding="utf-8") as f:
            json.dump(self.get_bindings(), f, indent=4)

    def load_from_file(self, file_path: str | Path) -> None:
        """
        Load bindings from a JSON file, replacing the current set.

        Unknown actions (not present in the defaults) are skipped. Known
        actions that are missing from the file keep their current key. The
        loaded bindings are normalized, persisted to ``app_state.json``, and
        a single ``bindings_changed`` signal is emitted at the end.

        Parameters
        ----------
        file_path
            The path to read from.

        Raises
        ------
        TypeError
            If the file does not contain a valid JSON object.
        """

        path = Path(file_path)
        with open(path, mode="r", encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, dict):
            msg = f"Keybind file {path} does not contain a JSON object."
            raise TypeError(msg)

        new_bindings: dict[str, dict[str, str]] = {
            action: dict(info) for action, info in self._bindings.items()
        }
        for action, info in data.items():
            if action not in new_bindings or not isinstance(info, dict):
                continue
            
            entry = dict(new_bindings[action])
            if isinstance(info.get("key"), str):
                entry["key"] = normalize_key_string(info["key"])
            
            new_bindings[action] = entry

        self._bindings = new_bindings
        self._persist()
        self.bindings_changed.emit(self.get_bindings())

    def register_handler(self, action: str, handler: Callable[[], None]) -> None:
        """
        Register a callable to be invoked when an action is triggered.

        Parameters
        ----------
        action
            The action key. Must be a known binding.
        handler
            A callable taking no arguments, invoked when the action fires.
        """

        if action not in self._bindings:
            print(f"[Warning] Cannot register handler for unknown action '{action}'.")
            return
        
        self._handlers[action] = handler

    def trigger(self, action: str) -> None:
        """
        Invoke the registered handler for an action, if one exists.

        Parameters
        ----------
        action
            The action key to trigger.
        """

        handler = self._handlers.get(action)
        if handler is None:
            print(f"[Warning] No handler registered for action '{action}'.")
            return
        
        handler()

    def to_qkeysequence(self, action: str) -> QKeySequence:
        """
        Build a ``QKeySequence`` from the key string for an action.

        Parameters
        ----------
        action
            The action key.

        Returns
        -------
        QKeySequence
            A ``QKeySequence`` constructed from the action's key string. Empty
            if the action is unknown or has no key.
        """

        key = self.get_key(action)
        if key is None:
            return QKeySequence()
        
        return QKeySequence(key, QKeySequence.SequenceFormat.PortableText)

    def to_frontend_dict(self) -> dict[str, str]:
        """
        Build a ``{ action: key }`` dict for the map-scope actions.

        This is the shape the TypeScript frontend expects when Python pushes
        the configured bindings into the ``window.map`` bridge.

        Returns
        -------
        dict[str, str]
            A dict of ``action -> normalized key string`` for map-scope actions.
        """

        return {
            action: info["key"]
            for action, info in self.get_actions_by_scope("map").items()
            if isinstance(info.get("key"), str) and info.get("key")
        }

    def _persist(self) -> None:
        """Write the current bindings back to the state file."""

        constants.SM.write("keybindings", self._bindings)


def qt_key_to_string(key: Qt.Key, modifiers: Qt.KeyboardModifier) -> str:
    """
    Convert a Qt key + modifiers into a normalized combination string.

    Used by the key-capture dialog to turn a ``QKeyEvent`` into the stored
    string form (e.g. ``"Ctrl+Shift+P"``).

    Parameters
    ----------
    key
        The Qt key code (from ``QKeyEvent.key()``).
    modifiers
        The Qt modifier flags (from ``QKeyEvent.modifiers()``).

    Returns
    -------
    str
        The normalized key combination string, or an empty string if the key
        alone is a modifier (e.g. pressing just ``Shift``).
    """

    modifier_keys = {
        Qt.Key.Key_Shift,
        Qt.Key.Key_Control,
        Qt.Key.Key_Alt,
        Qt.Key.Key_Meta,
    }
    if key in modifier_keys:
        return ""

    parts: list[str] = []
    if modifiers & Qt.KeyboardModifier.ControlModifier:
        parts.append("Ctrl")
    
    if modifiers & Qt.KeyboardModifier.ShiftModifier:
        parts.append("Shift")
    
    if modifiers & Qt.KeyboardModifier.AltModifier:
        parts.append("Alt")
    
    if modifiers & Qt.KeyboardModifier.MetaModifier:
        parts.append("Meta")

    parts.append(QKeySequence(key).toString().title())

    return normalize_key_string("+".join(parts))


# expose the helper for type-checkers that want the Any-typed version
def qt_key_event_to_string(event: QKeyEvent) -> str:
    """Convert a ``QKeyEvent`` into a normalized combination string.

    Parameters
    ----------
    event
        A ``QKeyEvent`` instance.

    Returns
    -------
    str
        The normalized key combination string, or an empty string if the event
        is a bare modifier press.
    """

    return qt_key_to_string(Qt.Key(event.key()), event.modifiers())
