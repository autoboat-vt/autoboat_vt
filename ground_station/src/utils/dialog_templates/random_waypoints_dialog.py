"""Dialog for generating random waypoints around the boat."""

from __future__ import annotations

from qtpy.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

__all__ = ["RandomWaypointsDialog"]


class RandomWaypointsDialog(QDialog):
    """
    A dialog for configuring a batch of randomly placed waypoints.

    The waypoints are scattered on a disc of the chosen radius around the boat's
    current position, which keeps them relevant to the mission instead of spreading
    them over the whole planet.

    Inherits
    --------
    :class:`QDialog`
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Add Random Waypoints")

        main_layout = QVBoxLayout(self)

        description = QLabel(
            "Waypoints are placed at random inside a circle centered on the boat's current position. "
            "The number of waypoints and the maximum distance from the boat can be configured below."
        )
        description.setWordWrap(True)
        main_layout.addWidget(description)

        form = QFormLayout()

        self._count_spinbox = QSpinBox()
        self._count_spinbox.setRange(1, 10_000)
        self._count_spinbox.setValue(10)
        self._count_spinbox.setToolTip("How many waypoints to generate.")
        form.addRow("Number of waypoints:", self._count_spinbox)

        self._distance_spinbox = QSpinBox()
        self._distance_spinbox.setRange(1, 1_000_000)
        self._distance_spinbox.setValue(100)
        self._distance_spinbox.setSuffix(" m")
        self._distance_spinbox.setToolTip("Maximum distance from the boat that a waypoint may be placed.")
        form.addRow("Maximum distance:", self._distance_spinbox)

        main_layout.addLayout(form)

        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        main_layout.addWidget(button_box)

    def get_count(self) -> int:
        """
        Get the requested number of waypoints.

        Returns
        -------
        `int`
            The number of waypoints the user wants to generate.
        """

        return self._count_spinbox.value()

    def get_max_distance_m(self) -> float:
        """
        Get the requested maximum distance from the boat.

        Returns
        -------
        `float`
            The maximum distance in meters that a generated waypoint may be from the boat.
        """

        return float(self._distance_spinbox.value())
