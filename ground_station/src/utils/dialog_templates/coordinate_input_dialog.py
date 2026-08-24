"""Dialog for entering coordinates in various formats."""

from __future__ import annotations

from itertools import pairwise

from qtpy.QtCore import QEvent, QObject, Qt
from qtpy.QtGui import QShowEvent
from qtpy.QtWidgets import (
    QComboBox,
    QDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from utils import misc

from .custom_buttons_dialog import show_message_box

__all__ = ["CoordinateInputDialog"]


class CoordinateInputDialog(QDialog):
    """
    A dialog for entering a latitude/longitude pair in several coordinate formats.

    Supported formats:
    - Decimal Degrees (e.g. ``37.7749, -122.4194``)
    - Degrees Decimal Minutes (e.g. ``37 46.4933 N, 122 24.5856 W``)
    - Degrees Minutes Seconds (e.g. ``37 46 29.6 N, 122 24 58.6 W``)

    Inherits
    --------
    :class:`QDialog`
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Manual Coordinate Entry")
        # these maps make it easier to navigate between fields with arrow keys
        self._down_map: dict[QWidget, QWidget] = {}
        self._up_map: dict[QWidget, QWidget] = {}
        self._left_map: dict[QWidget, QWidget] = {}
        self._right_map: dict[QWidget, QWidget] = {}

        main_layout = QVBoxLayout(self)

        self._stack = QStackedWidget()
        self._stack.addWidget(self._build_dd_page())
        self._stack.addWidget(self._build_ddm_page())
        self._stack.addWidget(self._build_dms_page())

        format_row = QHBoxLayout()
        format_row.addWidget(QLabel("Format:"))
        self._format_combo = QComboBox()
        self._format_combo.addItems(["Decimal Degrees", "Degrees Decimal Minutes", "Degrees Minutes Seconds"])
        self._format_combo.currentIndexChanged.connect(self._on_format_changed)
        format_row.addWidget(self._format_combo, stretch=1)
        main_layout.addLayout(format_row)

        main_layout.addWidget(self._stack)

        button_row = QHBoxLayout()
        button_row.addStretch(1)
        cancel_button = misc.pushbutton_maker(
            button_text="Cancel",
            function=self.reject,
            icon=None,
            min_height=30,
        )
        ok_button = misc.pushbutton_maker(
            button_text="Add Waypoint",
            function=self._on_accept,
            icon=None,
            min_height=30,
        )
        ok_button.setDefault(True)

        button_row.addWidget(cancel_button)
        button_row.addWidget(ok_button)
        main_layout.addLayout(button_row)

    def showEvent(self, event: QShowEvent) -> None:
        """Focus the first input field when the dialog is shown."""

        super().showEvent(event)
        self._focus_first_field()

    def _on_format_changed(self, index: int) -> None:
        """Switch the stacked page and focus the first field of the new format."""

        self._stack.setCurrentIndex(index)
        self._focus_first_field()

    def _focus_first_field(self) -> None:
        """Focus the first input field of the currently visible page."""

        idx = self._stack.currentIndex()
        if idx == 0:
            field = self._dd_lat
        elif idx == 1:
            field = self._ddm_lat_deg
        else:
            field = self._dms_lat_deg
        
        field.setFocus()
        if isinstance(field, QLineEdit):
            field.setCursorPosition(len(field.text()))

    def _build_dd_page(self) -> QWidget:
        """Decimal Degrees page with two text fields for latitude and longitude."""

        page = QWidget()
        form = QFormLayout(page)
        self._dd_lat = self._make_text_field("0.0", "Latitude in decimal degrees")
        self._dd_lon = self._make_text_field("0.0", "Longitude in decimal degrees")
        form.addRow("Latitude (°):", self._dd_lat)
        form.addRow("Longitude (°):", self._dd_lon)
        self._register_fields([[self._dd_lat], [self._dd_lon]])
        return page

    def _build_ddm_page(self) -> QWidget:
        """Degrees + decimal minutes + hemisphere selector."""

        page = QWidget()
        form = QFormLayout(page)

        self._ddm_lat_deg = self._make_text_field("0", "Latitude degrees")
        self._ddm_lat_min = self._make_text_field("0.0", "Latitude minutes")
        self._ddm_lat_hemi = self._make_combo(["N", "S"])
        lat_row = QHBoxLayout()
        lat_row.addWidget(self._ddm_lat_deg)
        lat_row.addWidget(QLabel("°"))
        lat_row.addWidget(self._ddm_lat_min)
        lat_row.addWidget(QLabel("'"))
        lat_row.addWidget(self._ddm_lat_hemi)
        form.addRow("Latitude:", lat_row)

        self._ddm_lon_deg = self._make_text_field("0", "Longitude degrees")
        self._ddm_lon_min = self._make_text_field("0.0", "Longitude minutes")
        self._ddm_lon_hemi = self._make_combo(["E", "W"])
        lon_row = QHBoxLayout()
        lon_row.addWidget(self._ddm_lon_deg)
        lon_row.addWidget(QLabel("°"))
        lon_row.addWidget(self._ddm_lon_min)
        lon_row.addWidget(QLabel("'"))
        lon_row.addWidget(self._ddm_lon_hemi)
        form.addRow("Longitude:", lon_row)

        self._register_fields([
            [self._ddm_lat_deg, self._ddm_lat_min, self._ddm_lat_hemi],
            [self._ddm_lon_deg, self._ddm_lon_min, self._ddm_lon_hemi]
        ])
        
        return page

    def _build_dms_page(self) -> QWidget:
        """Degrees + minutes + seconds + hemisphere selector."""

        page = QWidget()
        form = QFormLayout(page)

        self._dms_lat_deg = self._make_text_field("0", "Latitude degrees")
        self._dms_lat_min = self._make_text_field("0", "Latitude minutes")
        self._dms_lat_sec = self._make_text_field("0.0", "Latitude seconds")
        self._dms_lat_hemi = self._make_combo(["N", "S"])
        lat_row = QHBoxLayout()
        lat_row.addWidget(self._dms_lat_deg)
        lat_row.addWidget(QLabel("°"))
        lat_row.addWidget(self._dms_lat_min)
        lat_row.addWidget(QLabel("'"))
        lat_row.addWidget(self._dms_lat_sec)
        lat_row.addWidget(QLabel('"'))
        lat_row.addWidget(self._dms_lat_hemi)
        form.addRow("Latitude:", lat_row)

        self._dms_lon_deg = self._make_text_field("0", "Longitude degrees")
        self._dms_lon_min = self._make_text_field("0", "Longitude minutes")
        self._dms_lon_sec = self._make_text_field("0.0", "Longitude seconds")
        self._dms_lon_hemi = self._make_combo(["E", "W"])
        lon_row = QHBoxLayout()
        lon_row.addWidget(self._dms_lon_deg)
        lon_row.addWidget(QLabel("°"))
        lon_row.addWidget(self._dms_lon_min)
        lon_row.addWidget(QLabel("'"))
        lon_row.addWidget(self._dms_lon_sec)
        lon_row.addWidget(QLabel('"'))
        lon_row.addWidget(self._dms_lon_hemi)
        form.addRow("Longitude:", lon_row)

        self._register_fields([
            [self._dms_lat_deg, self._dms_lat_min, self._dms_lat_sec, self._dms_lat_hemi],
            [self._dms_lon_deg, self._dms_lon_min, self._dms_lon_sec, self._dms_lon_hemi]
        ])
        
        return page

    @staticmethod
    def _make_text_field(default: str = "0", tooltip: str = "") -> QLineEdit:
        """A text field for numeric coordinate entry."""

        field = QLineEdit(default)
        field.setClearButtonEnabled(True)
        field.setToolTip(tooltip)
        return field

    @staticmethod
    def _make_combo(items: list[str]) -> QComboBox:
        """A hemisphere combo box."""

        combo = QComboBox()
        combo.addItems(items)
        return combo

    def _register_fields(self, rows: list[list[QWidget]]) -> None:
        """
        Install the arrow-key navigation filter, set tab order, and build a vertical map.

        Parameters
        ----------
        rows
            A list of rows, where each row is a list of fields in left-to-right order.
        """

        flat: list[QWidget] = []
        for row in rows:
            flat.extend(row)
            for field in row:
                field.installEventFilter(self)
            
            for prev, curr in pairwise(row):
                self.setTabOrder(prev, curr)
        
        for prev_row, curr_row in pairwise(rows):
            self.setTabOrder(prev_row[-1], curr_row[0])

        self._down_map.update(
            {upper: lower for row_pair in pairwise(rows) for upper, lower in zip(row_pair[0], row_pair[1], strict=True)}
        )
        self._up_map.update(
            {lower: upper for row_pair in pairwise(rows) for upper, lower in zip(row_pair[0], row_pair[1], strict=True)}
        )
        
        for row in rows:
            self._right_map.update(pairwise(row))
            self._left_map.update({b: a for a, b in pairwise(row)})

    def eventFilter(self, obj: QObject | None, event: QEvent | None) -> bool:
        """
        Move focus between fields on arrow key presses.

        Parameters
        ----------
        obj
            The object that received the event.
        event
            The event that was received.

        Returns
        -------
        bool
            ``True`` if the event was handled, ``False`` otherwise.
        """

        if event is not None and event.type() == QEvent.Type.KeyPress and obj is not None:
            key = event.key()
            if key in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
                target = None
                if key == Qt.Key.Key_Down:
                    target = self._down_map.get(obj)
                
                elif key == Qt.Key.Key_Up:
                    target = self._up_map.get(obj)
                
                elif key == Qt.Key.Key_Right:
                    target = self._right_map.get(obj) if (
                        not isinstance(obj, QLineEdit) or obj.cursorPosition() >= len(obj.text())
                    ) else None
                
                elif key == Qt.Key.Key_Left:
                    target = self._left_map.get(obj) if (
                        not isinstance(obj, QLineEdit) or obj.cursorPosition() <= 0
                    ) else None
                
                if target is not None:
                    target.setFocus()
                    if isinstance(target, QLineEdit):
                        if key in (Qt.Key.Key_Left, Qt.Key.Key_Up):
                            target.setCursorPosition(0)
                        else:
                            target.setCursorPosition(len(target.text()))
                    
                    return True
        
        return super().eventFilter(obj, event)

    @staticmethod
    def _dms_to_decimal(deg: float, minutes: float, seconds: float, positive: bool) -> float:
        """
        Convert degrees, minutes, and seconds to decimal degrees.

        Parameters
        ----------
        deg
            The degrees component.
        minutes
            The minutes component.
        seconds
            The seconds component.
        positive
            Whether the resulting decimal degrees should be positive (True) or negative (False).

        Returns
        -------
        float
            The decimal degrees value.
        """

        value = abs(deg) + minutes / 60.0 + seconds / 3600.0
        return value if positive else -value

    def _on_accept(self) -> None:
        """Validate the current page and accept the dialog if valid."""

        try:
            self.get_coordinates()
        except (ValueError, OverflowError) as exc:
            show_message_box(title="Invalid Coordinates", message=str(exc))
            return

        self.accept()

    @staticmethod
    def _parse_float(field: QLineEdit, name: str) -> float:
        """Parse a text field's contents into a float."""

        text = field.text().strip()
        if not text:
            raise ValueError(f"{name} is empty.")
        try:
            return float(text)
        except ValueError as exc:
            raise ValueError(f"{name} '{text}' is not a valid number.") from exc

    def get_coordinates(self) -> tuple[float, float]:
        """
        Return the entered coordinates as decimal degrees.

        Returns
        -------
        tuple[float, float]
            ``(latitude, longitude)`` in decimal degrees.

        Raises
        ------
        ValueError
            If the values are out of range for the selected format.
        """

        idx = self._stack.currentIndex()
        # decimal degrees
        if idx == 0:
            lat = self._parse_float(self._dd_lat, "Latitude")
            lon = self._parse_float(self._dd_lon, "Longitude")
        # degrees decimal minutes
        elif idx == 1:
            lat = self._dms_to_decimal(
                self._parse_float(self._ddm_lat_deg, "Latitude degrees"),
                self._parse_float(self._ddm_lat_min, "Latitude minutes"),
                0.0,
                self._ddm_lat_hemi.currentText() == "N",
            )
            lon = self._dms_to_decimal(
                self._parse_float(self._ddm_lon_deg, "Longitude degrees"),
                self._parse_float(self._ddm_lon_min, "Longitude minutes"),
                0.0,
                self._ddm_lon_hemi.currentText() == "E",
            )
        
        # degrees minutes seconds
        else:
            lat = self._dms_to_decimal(
                self._parse_float(self._dms_lat_deg, "Latitude degrees"),
                self._parse_float(self._dms_lat_min, "Latitude minutes"),
                self._parse_float(self._dms_lat_sec, "Latitude seconds"),
                self._dms_lat_hemi.currentText() == "N",
            )
            lon = self._dms_to_decimal(
                self._parse_float(self._dms_lon_deg, "Longitude degrees"),
                self._parse_float(self._dms_lon_min, "Longitude minutes"),
                self._parse_float(self._dms_lon_sec, "Longitude seconds"),
                self._dms_lon_hemi.currentText() == "E",
            )

        if not -90.0 <= lat <= 90.0:
            raise ValueError(f"Latitude {lat} is out of range (-90 to 90).")
        
        if not -180.0 <= lon <= 180.0:
            raise ValueError(f"Longitude {lon} is out of range (-180 to 180).")

        return lat, lon
