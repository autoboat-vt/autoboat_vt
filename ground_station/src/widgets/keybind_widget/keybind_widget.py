from __future__ import annotations

import time
from pathlib import Path

from qtpy.QtCore import QPoint, Qt, Slot
from qtpy.QtGui import QKeyEvent
from qtpy.QtWidgets import (
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMenu,
    QSizePolicy,
    QSpacerItem,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
)

from utils import constants, misc

from .keybind_manager import get_keybind_manager, qt_key_event_to_string

__all__ = ["KeybindConfigDialog"]


class KeybindConfigWidget(QTableWidget):
    """
    A table widget that captures key presses for rebinding.

    While a ```KeyCaptureItem``` cell has focus, the next key press is
    interpreted as a new binding rather than a normal table navigation event.

    Parameters
    ----------
    rows
        The number of rows in the table.

    Inherits
    -------
    ``QTableWidget``
    """

    def __init__(self, rows: int) -> None:
        super().__init__(rows, 4)

    UNBOUND_DISPLAY = "— Unbound —"

    def keyPressEvent(self, event: QKeyEvent) -> None:
        """
        Capture a key press if a key-capture cell is focused.

        ``Delete`` and ``Backspace`` clear the binding instead of capturing.

        Parameters
        ----------
        event
            The key event to handle.
        """

        current = self.currentItem()
        if isinstance(current, KeyCaptureItem) and current.listening:
            if event.key() in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
                current.set_combo("")
                return

            combo = qt_key_event_to_string(event)
            if combo:
                current.set_combo(combo)
                return

        super().keyPressEvent(event)


class KeyCaptureItem(QTableWidgetItem):
    """
    A table item that displays the current keybind and captures a new one.

    When the user clicks the item (or it gains focus), it enters "listening"
    mode and displays a prompt. The next captured key press updates the
    binding via the ```KeybindManager``` and exits listening mode.

    Parameters
    ----------
    action
        The action key this item binds (e.g. "focus_boat").
    initial_key
        The initial key combination string to display.

    Inherits
    -------
    ``QTableWidgetItem``
    """

    __slots__ = ("_action", "_listening", "_combo")

    def __init__(self, action: str, initial_key: str) -> None:
        super().__init__(initial_key or KeybindConfigWidget.UNBOUND_DISPLAY)
        
        self._action = action
        self._listening = False
        self._combo = initial_key
        self.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

    @property
    def action(self) -> str:
        """The action key this item binds."""

        return self._action

    @property
    def listening(self) -> bool:
        """Whether this item is currently listening for a key press."""

        return self._listening

    @property
    def combo(self) -> str:
        """The current key combination string."""

        return self._combo

    def start_listening(self) -> None:
        """Put the item into listening mode, displaying a prompt."""

        self._listening = True
        self.setText("Press a key... (Delete to clear)")

    def cancel_listening(self) -> None:
        """Exit listening mode and restore the displayed combo."""

        self._listening = False
        self.setText(self._display_text())

    def _display_text(self) -> str:
        """Return the user-facing text for the current combo."""

        return self._combo or KeybindConfigWidget.UNBOUND_DISPLAY

    def set_combo(self, combo: str) -> None:
        """
        Set the combo, persist it, and exit listening mode.

        Parameters
        ----------
        combo
            The new key combination string (already normalized by the caller).
            An empty string clears the binding.
        """

        self._combo = combo
        self._listening = False
        self.setText(self._display_text())

        manager = get_keybind_manager()
        conflict = manager.set_key(self._action, combo)
        
        table = self.tableWidget()
        if table is not None and hasattr(table, "parent") and isinstance(table.parent(), KeybindConfigDialog):
            table.parent()._handle_conflict(self._action, conflict)


class KeybindConfigDialog(QDialog):
    """
    Modal dialog for viewing and editing keybinds.

    Parameters
    ----------
    embedded
        When ``True`` (e.g. used as a tab), the dialog omits the window title
        and the Close button and uses an expanding size policy. When
        ``False`` (standalone dialog), it gets a fixed size and a Close
        button.

    Inherits
    -------
    ``QDialog``
    """

    table_style = """
    QTableWidget {
        gridline-color: #444444;
        selection-background-color: transparent;
        selection-color: white;
    }

    QTableWidget::item {
        color: white;
        background-color: transparent;
    }

    QTableWidget::item:selected {
        background-color: transparent;
        color: white;
    }

    QTableWidget::item:focus {
        background-color: transparent;
        color: white;
    }

    QHeaderView::section {
        padding: 4px;
    }
    """

    feedback_text_style = """
    QTextEdit {
        color: white;
        background-color: transparent;
        border: 1px solid white;
    }
    """

    def __init__(self, embedded: bool = False) -> None:
        super().__init__()

        self._embedded = embedded
        if not embedded:
            self.setWindowTitle("Keybind Configuration")
            self.setModal(True)
        
        self.layout = QVBoxLayout(self)

        manager = get_keybind_manager()
        self._bindings_snapshot = manager.get_bindings()

        self._table = KeybindConfigWidget(len(self._bindings_snapshot))
        self._table.setStyleSheet(self.table_style)
        self._table.setHorizontalHeaderLabels(["Scope", "Action", "Key", "Description"])
        self._table.verticalHeader().setVisible(False)
        self._table.setAlternatingRowColors(False)
        self._table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectItems)
        self._table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_context_menu)
        self._table.cellClicked.connect(self._on_cell_clicked)

        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)

        self._key_items: dict[str, KeyCaptureItem] = {}

        for row, (action, info) in enumerate(self._bindings_snapshot.items()):
            self._add_action_row(row=row, action=action, info=info)

        self.feedback_text_default = "Click a key cell, then press a key combination to rebind it."
        self.feedback_text_label = QLabel("Additional Information:")

        self.feedback_text = QTextEdit(self)
        self.feedback_text.setReadOnly(True)
        self.feedback_text.setStyleSheet(self.feedback_text_style)
        self.feedback_text.setText(self.feedback_text_default)
        self.feedback_text_label.setBuddy(self.feedback_text)

        self.feedback_text_clear_timer = misc.copy_qtimer(constants.FIVE_SECOND_TIMER)
        self.feedback_text_clear_timer.setSingleShot(True)
        self.feedback_text_clear_timer.timeout.connect(self.clear_feedback_text)

        manager.bindings_changed.connect(self._on_bindings_changed)

        button_row = QHBoxLayout()
        self.reset_button = misc.pushbutton_maker(
            "Reset to Defaults",
            self._on_reset_clicked,
            constants.ICONS.refresh,
            min_height=30,
        )
        self.save_button = misc.pushbutton_maker(
            "Save to File",
            self._on_save_clicked,
            constants.ICONS.save,
            min_height=30,
        )
        self.load_button = misc.pushbutton_maker(
            "Load from File",
            self._on_load_clicked,
            constants.ICONS.hard_drive,
            min_height=30,
        )
        
        # adding stretch to both sides -> buttons are centered
        button_row.addStretch()
        button_row.addWidget(self.reset_button)
        button_row.addWidget(self.save_button)
        button_row.addWidget(self.load_button)
        button_row.addStretch()

        self.layout.addWidget(self._table)
        self.layout.addItem(QSpacerItem(0, 12, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed))
        self.layout.addWidget(self.feedback_text_label)
        self.layout.addWidget(self.feedback_text)
        self.layout.addLayout(button_row)

        self._table.resizeColumnsToContents()
        self._table.resizeRowsToContents()

        table_width = (
            self._table.verticalHeader().width()
            + self._table.frameWidth() * 2
            + sum(self._table.columnWidth(i) for i in range(self._table.columnCount()))
        )

        table_height = (
            self._table.horizontalHeader().height()
            + self._table.frameWidth() * 2
            + sum(self._table.rowHeight(i) for i in range(self._table.rowCount()))
        )

        self._table.setFixedSize(table_width, table_height)

        if not embedded:
            self.adjustSize()
            self.setFixedSize(self.size())
        else:
            self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def _add_action_row(self, row: int, action: str, info: dict[str, str]) -> None:
        """
        Add a row to the table for a single action.

        Parameters
        ----------
        row
            The row index.
        action
            The action key.
        info
            The binding info dict (name, scope, key, description).
        """

        scope_item = QTableWidgetItem(info.get("scope", ""))
        scope_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self._table.setItem(row, 0, scope_item)

        name_item = QTableWidgetItem(info.get("name", action))
        name_item.setToolTip(info.get("description", ""))
        self._table.setItem(row, 1, name_item)

        key_item = KeyCaptureItem(action=action, initial_key=info.get("key", ""))
        self._table.setItem(row, 2, key_item)
        self._key_items[action] = key_item

        description_item = QTableWidgetItem(info.get("description", ""))
        description_item.setToolTip(info.get("description", ""))
        self._table.setItem(row, 3, description_item)

    @Slot(int, int)
    def _on_cell_clicked(self, row: int, column: int) -> None:
        """
        Handle a click on the table. If the key column was clicked, start listening.

        Parameters
        ----------
        row
            The clicked row.
        column
            The clicked column.
        """

        if column != 2:
            return

        item = self._table.item(row, column)
        if not isinstance(item, KeyCaptureItem):
            return

        # cancel any other listening items
        for other in self._key_items.values():
            if other is not item and other.listening:
                other.cancel_listening()

        item.start_listening()
        self.update_feedback_text(f"Press a new key for '{item.action}'. Delete to clear. Esc to cancel.")

    @Slot(object)
    def _on_context_menu(self, position: QPoint) -> None:
        """
        Show a context menu on right-click of a key cell.

        Offers a "Clear Keybind" action when the targeted binding has a key set.

        Parameters
        ----------
        position
            The position in table coordinates where the right-click occurred.
        """

        item = self._table.itemAt(position)
        if not isinstance(item, KeyCaptureItem):
            return

        menu = QMenu(self._table)
        clear_action = menu.addAction("Clear Keybind")
        clear_action.setEnabled(bool(item.combo))
        chosen = menu.exec(self._table.viewport().mapToGlobal(position))

        if chosen is clear_action:
            item.set_combo("")

    def _handle_conflict(self, action: str, conflict: str | None) -> None:
        """
        Handle a possible conflict after a binding is set.

        Parameters
        ----------
        action
            The action that was just rebound.
        conflict
            The action key of a conflicting binding, or ``None``.
        """

        if conflict is None:
            self.update_feedback_text(f"Updated binding for '{action}'.")
            return

        manager = get_keybind_manager()
        conflict_info = manager.get_binding(conflict)
        conflict_name = conflict_info.get("name", conflict) if conflict_info else conflict
        action_info = manager.get_binding(action)
        action_name = action_info.get("name", action) if action_info else action
        
        self.update_feedback_text(
            f"Note: '{conflict_name}' is also bound to this key. "
            f"Both '{action_name}' and '{conflict_name}' will fire."
        )

    @Slot(dict)
    def _on_bindings_changed(self, bindings: dict[str, dict[str, str]]) -> None:
        """
        Refresh the displayed keys when bindings change externally.

        Parameters
        ----------
        bindings
            The full bindings dict from the keybind manager.
        """

        self._bindings_snapshot = bindings
        for action, info in bindings.items():
            item = self._key_items.get(action)
            if item is not None and not item.listening:
                key = info.get("key", "")
                item._combo = key  # keep internal state in sync
                item.setText(item._display_text())

    @Slot()
    def _on_reset_clicked(self) -> None:
        """Reset all bindings to defaults."""

        get_keybind_manager().reset_to_defaults()
        self.update_feedback_text("All keybinds reset to defaults.")

    @Slot()
    def _on_save_clicked(self) -> None:
        """Save the current bindings to a JSON file chosen by the user."""

        manager = get_keybind_manager()
        default_path = Path(constants.KEYBINDS_DIR / f"keybinds_{time.time_ns()}.json")
        chosen = QFileDialog.getSaveFileName(
            self,
            "Save Keybinds",
            default_path.as_posix(),
            "JSON Files (*.json)",
        )
        file_path = chosen[0] if chosen and chosen[0] else ""
        if not file_path:
            self.update_feedback_text("Save cancelled.")
            return

        try:
            manager.save_to_file(file_path)
        
        except OSError as e:
            self.update_feedback_text(f"Failed to save: {e}")
            return

        self.update_feedback_text(f"Saved keybinds to {Path(file_path).name}.")

    @Slot()
    def _on_load_clicked(self) -> None:
        """Load bindings from a JSON file chosen by the user."""

        chosen = QFileDialog.getOpenFileName(
            self,
            "Load Keybinds",
            constants.KEYBINDS_DIR.as_posix(),
            "JSON Files (*.json)",
        )
        file_path = chosen[0] if chosen and chosen[0] else ""
        if not file_path:
            self.update_feedback_text("Load cancelled.")
            return

        manager = get_keybind_manager()
        try:
            manager.load_from_file(file_path)
        except (OSError, TypeError) as e:
            self.update_feedback_text(f"Failed to load: {e}")
            return

        self.update_feedback_text(f"Loaded keybinds from {Path(file_path).name}.")

    @Slot()
    def _on_close_clicked(self) -> None:
        """Close the dialog (cancelling any in-progress capture)."""

        for item in self._key_items.values():
            if item.listening:
                item.cancel_listening()
        
        self.close()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        """
        Handle Esc at the dialog level to cancel any listening capture.

        Parameters
        ----------
        event
            The key event.
        """

        if event.key() == Qt.Key.Key_Escape:
            for item in self._key_items.values():
                if item.listening:
                    item.cancel_listening()
                    self.update_feedback_text("Cancelled.")
                    return
        
        super().keyPressEvent(event)

    @Slot(str)
    def update_feedback_text(self, text: str) -> None:
        """Update the feedback text in the dialog.

        Parameters
        ----------
        text
            The new feedback text to display.
        """

        self.feedback_text.setPlainText(text)
        self.feedback_text_clear_timer.start()

    @Slot()
    def clear_feedback_text(self) -> None:
        """Reset the feedback text to the default message."""

        self.feedback_text.setPlainText(self.feedback_text_default)
