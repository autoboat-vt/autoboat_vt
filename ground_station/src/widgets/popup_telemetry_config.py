from __future__ import annotations

from collections.abc import Callable
from functools import partial

from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QCheckBox,
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from utils import constants, misc


class EditTelemetryConfigWindow(QDialog):
    """
    A dialog for controlling which diagnostics appear on the map.

    Parameters
    ----------
    waypoints_checker_callback
        Called when the waypoints popup setting changes.
    debugging_symbols_callback
        Called when the debugging symbols setting changes.

    Inherits
    -------
    ``QDialog``
    """

    checkbox_style = """
    QCheckBox {
        color: white;
    }
    """

    feedback_text_style = """
    QTextEdit {
        color: white;
        background-color: transparent;
        border: 1px solid white;
    }
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

    def __init__(
        self,
        waypoints_checker_callback: Callable[[bool], None],
        debugging_symbols_callback: Callable[[bool], None],
    ) -> None:
        super().__init__()

        self.setWindowTitle("Map Appearance Configuration")
        self.layout = QVBoxLayout(self)

        self.feature_table = QTableWidget(2, 3, self)
        self.feature_table.setHorizontalHeaderLabels(["Feature Name", "Description", "Enabled"])
        self.feature_table.setStyleSheet(self.table_style)
        self.feature_table.verticalHeader().setVisible(False)
        self.feature_table.setAlternatingRowColors(False)
        self.feature_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.feature_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.feature_table.setCurrentCell(-1, -1)
        self.feature_table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.feature_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)

        header = self.feature_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)

        self.feedback_text_default = "Toggle a setting to see more information about it here."
        self.feedback_text_label = QLabel("Additional Information:")

        self.feedback_text = QTextEdit(self)
        self.feedback_text.setReadOnly(True)
        self.feedback_text.setStyleSheet(EditTelemetryConfigWindow.feedback_text_style)
        self.feedback_text.setText(self.feedback_text_default)
        self.feedback_text_label.setBuddy(self.feedback_text)

        self.feedback_text_clear_timer = misc.copy_qtimer(constants.FIVE_SECOND_TIMER)
        self.feedback_text_clear_timer.setSingleShot(True)
        self.feedback_text_clear_timer.timeout.connect(self.clear_feedback_text)

        self._add_feature_row(
            row=0,
            name="Waypoints Popup",
            description="Show a popup when the waypoints on the telemetry server change.",
            callback=waypoints_checker_callback,
            feedback_text="Updated Waypoints Popup Config.",
            checked=False,
        )

        self._add_feature_row(
            row=1,
            name="Debugging Symbols",
            description="Show sailboat debugging symbols on the map.",
            callback=debugging_symbols_callback,
            feedback_text=(
                "Updated Debugging Symbols Config.\n"
                "orange, wind\n"
                "black, velocity\n"
                "red, no-go zone\n"
                "pink, decision zone 2"
            ),
            checked=False,
        )

        button_row = QHBoxLayout()
        self.close_button = QPushButton("Close", self)
        self.close_button.clicked.connect(self.accept)
        button_row.addStretch()
        button_row.addWidget(self.close_button)
        button_row.addStretch()

        self.layout.addWidget(self.feature_table)
        self.layout.addItem(QSpacerItem(0, 12, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed))
        self.layout.addWidget(self.feedback_text_label)
        self.layout.addWidget(self.feedback_text)
        self.layout.addLayout(button_row)

        self.feature_table.resizeColumnsToContents()
        self.feature_table.resizeRowsToContents()

        table_width = (
            self.feature_table.verticalHeader().width()
            + self.feature_table.frameWidth() * 2
            + sum(self.feature_table.columnWidth(i) for i in range(self.feature_table.columnCount()))
        )

        table_height = (
            self.feature_table.horizontalHeader().height()
            + self.feature_table.frameWidth() * 2
            + sum(self.feature_table.rowHeight(i) for i in range(self.feature_table.rowCount()))
        )

        self.feature_table.setFixedSize(table_width, table_height)
        self.adjustSize()
        self.setFixedSize(self.size())

    def _add_feature_row(
        self,
        row: int,
        name: str,
        description: str,
        callback: Callable[[bool], None],
        feedback_text: str,
        checked: bool,
    ) -> None:
        """
        Add a row to the feature table for a diagnostic feature.
        
        Parameters
        ----------
        row
            The row index to add the feature to.
        name
            The feature name shown in the table.
        description
            A description of the feature.
        callback
            Called when the feature is enabled or disabled.
        feedback_text
            The text to show in the feedback box when the feature is toggled.
        checked
            Whether the feature is initially enabled.
        """

        name_item = FeatureInfoItem(
            name=name,
            description=description,
            callback=callback,
        )
        self.feature_table.setItem(row, 0, name_item)

        description_item = QTableWidgetItem(description)
        description_item.setToolTip(description)
        self.feature_table.setItem(row, 1, description_item)

        checkbox = QCheckBox(self)
        checkbox.setStyleSheet(EditTelemetryConfigWindow.checkbox_style)
        checkbox.setChecked(checked)
        checkbox.setToolTip(description)
        checkbox.toggled.connect(callback)
        checkbox.toggled.connect(partial(self.update_feedback_text, feedback_text))

        checkbox_container = QWidget(self)
        checkbox_layout = QHBoxLayout(checkbox_container)
        checkbox_layout.setContentsMargins(8, 0, 8, 0)
        checkbox_layout.addWidget(checkbox)
        checkbox_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.feature_table.setCellWidget(row, 2, checkbox_container)

    def update_feedback_text(self, text: str) -> None:
        """
        Update the feedback text in the dialog.
        
        Parameters
        ----------
        text
            The new feedback text to display.
        """

        self.feedback_text.setPlainText(text)
        self.feedback_text_clear_timer.start()

    def clear_feedback_text(self) -> None:
        """Clear the feedback text, resetting it to the default message."""

        self.feedback_text.setPlainText(self.feedback_text_default)

class FeatureInfoItem(QTableWidgetItem):
    """
    A table item that stores information about a diagnostic feature.

    Parameters
    ----------
    name
        The feature name shown in the table.
    description
        A description of the feature.
    callback
        Called when the feature is enabled or disabled.
        Used to communicate with the ``GroundStation`` widget to update the telemetry configuration.

    Inherits
    -------
    ``QTableWidgetItem``
    """

    def __init__(
        self,
        name: str,
        description: str,
        callback: Callable[[bool], None],
    ) -> None:
        super().__init__(name)
        self.description = description
        self.callback = callback
        self.setToolTip(description)
