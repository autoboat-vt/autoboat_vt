import time
from collections import deque
from typing import Any
from urllib.parse import urljoin

import numpy as np
import numpy.typing as npt
import pyqtgraph as pg
from qtpy.QtCore import Qt, Signal
from qtpy.QtWidgets import QCheckBox, QDialog, QGridLayout, QWidget
from requests.exceptions import RequestException
from utils import constants, misc
from utils.thread_classes import BoatStatusThreadRouter


class GraphViewer(QWidget):
    """
    A widget for monitoring data in real-time using graphs.

    Attributes
    ----------
    boat_data_signal
        A signal that emits the latest boat data as a tuple containing a dictionary
        of boat status and a ``TelemetryStatus`` enum value.

    Inherits
    -------
    ``QWidget``
    """

    boat_data_signal = Signal(tuple)

    def __init__(self) -> None:
        super().__init__()

        self.timer = misc.copy_qtimer(constants.ONE_SECOND_TIMER)
        self.plots: list[pg.PlotItem] = []

        self.time_stopped: float | None = None
        self.time_started: float | None = None

        self.important_keys: list[str] = ["speed", "distance_to_next_waypoint", "desired_heading", "heading", "true_wind_speed"]
        self.available_keys: list[str] = []

        self.history_length: int = 200
        self.x_axis: deque[np.float64] = deque(maxlen=self.history_length)
        self.data: dict[str, deque[npt.NDArray[np.float64]]] = {}

        self.main_layout = QGridLayout()

        self.graph_layout_widget = pg.GraphicsLayoutWidget()
        self.main_layout.addWidget(self.graph_layout_widget, 0, 0, 1, 2)

        self.open_graphs_dialog_button = misc.pushbutton_maker("Select Graphs", constants.ICONS.cog, self.select_graphs)
        self.clear_graphs_button = misc.pushbutton_maker("Clear Graphs", constants.ICONS.refresh, self.clear_graphs)

        self.main_layout.addWidget(self.clear_graphs_button, 1, 0)
        self.main_layout.addWidget(self.open_graphs_dialog_button, 1, 1)
        self.setLayout(self.main_layout)

        self.telemetry_handler = BoatStatusThreadRouter.BoatStatusFetcherThread()
        self.telemetry_handler.response.connect(self.update_graph)
        self.telemetry_handler.start()

    def update_graph(self, request_result: tuple[dict[str, Any], constants.TelemetryStatus]) -> None:
        """Update the graphs with new telemetry data.

        Parameters
        ----------
        request_result
            A tuple containing:
                - a dictionary of boat status,
                - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        if constants.SM.read("has_telemetry_server_instance_changed"):
            self.x_axis.clear()
            self.data.clear()
            self.plots.clear()
            self.graph_layout_widget.clear()

        self.boat_data_signal.emit(request_result)
        boat_data, telemetry_status = request_result

        if telemetry_status is not constants.TelemetryStatus.SUCCESS:
            print("[Warning] Failed to fetch telemetry data for graphs, skipping update.")
            return

        try:
            self.available_keys = {
                key for key in boat_data if
                (
                    isinstance(boat_data[key], constants.NumberType)
                    and key != "current_waypoint_index"
                )
            }
            filtered_values = {key: float(boat_data.get(key, np.nan)) for key in self.important_keys}

            current_time = time.time() - constants.SM.read("start_time")
            self.x_axis.append(current_time)

            for key, val in filtered_values.items():
                if key not in self.data:
                    self.data[key] = deque(maxlen=self.history_length)
                self.data[key].append(np.array([val], dtype=np.float64))

            if not self.plots:
                num_keys = len(self.important_keys)
                last_is_odd = num_keys % 2 == 1

                for i, key in enumerate(self.data):
                    if last_is_odd and i == num_keys - 1:
                        plot_row = i // 2
                        plot_col = 0
                        col_span = 2
                    else:
                        plot_row = i // 2
                        plot_col = i % 2
                        col_span = 1

                    plot_title = key.replace("_", " ").title()
                    
                    plot_item = self.graph_layout_widget.addPlot(
                        row=plot_row, col=plot_col, colspan=col_span, title=plot_title
                    )
                    if not isinstance(plot_item, pg.PlotItem):
                        print(f"[Error] Failed to create plot for key '{key}', continuing with next key.")
                        continue

                    if col_span == 1:
                        plot_item.setPreferredWidth(400)

                    plot_item.setMouseEnabled(x=True, y=True)
                    plot_item.setLabel("bottom", "Time", "s")
                    plot_item.showGrid(x=True, y=True)

                    self.plots.append(plot_item)
                    curve = plot_item.plot(pen=pg.mkPen(color=(255, 0, 0)))

                    y = np.concatenate(list(self.data[key]))
                    x = list(self.x_axis)[-len(y) :]
                    curve.setData(x, y)

            else:
                for i, key in enumerate(self.data.keys()):
                    curve = self.plots[i].listDataItems()[0]
                    y = np.concatenate(list(self.data[key]))
                    x = list(self.x_axis)[-len(y) :]
                    curve.setData(x, y)

        except Exception as e:
            print(f"[Error] Failed to update graphs: {e}")

    def select_graphs(self) -> None:
        """Open a dialog to select which graphs to display."""

        if not self.available_keys:
            print("[Info] Pulling available boat data fields from telemetry server for graph selection...")
            try:
                response = constants.REQ_SESSION.get(
                    urljoin(
                        misc.get_route("get_boat_status"),
                        str(constants.SM.read("telemetry_server_instance_id"))
                    )
                ).json()

                if not isinstance(response, dict):
                    raise TypeError

                self.available_keys = {
                    k: v for k, v in response.items() if isinstance(v, (int, float)) and k != "current_waypoint_index"
                }
                print("[Info] Successfully fetched available boat data fields for graph selection.")

            except RequestException:
                print("[Error] Failed to connect to telemetry server to fetch available boat data fields for graph selection.")
                return

            except TypeError:
                print(f"[Error] Unexpected response format from telemetry server: {response}")
                return

        self.graph_dialog = GraphSelectionDialog(available_keys=self.available_keys, selected_keys=self.important_keys)
        self.graph_dialog.setWindowModality(Qt.ApplicationModal)
        self.graph_dialog.show()

        self.graph_dialog.apply_button.clicked.connect(lambda: self.apply_graph_selection(self.graph_dialog.selected_keys))

    def apply_graph_selection(self, selected_keys: list[str]) -> None:
        """
        Apply the selected graphs to display.

        Parameters
        ----------
        selected_keys
            A list of keys representing the selected graphs to display.
        """

        self.important_keys = selected_keys
        self.x_axis.clear()
        self.data.clear()
        self.plots.clear()
        self.graph_layout_widget.clear()
        print(f"[Info] Selected graphs updated: {', '.join(selected_keys)}")

    def clear_graphs(self) -> None:
        """Clear all graphs and data."""

        self.x_axis.clear()
        self.data.clear()
        self.plots.clear()
        self.graph_layout_widget.clear()
        print("[Info] Cleared all graphs and data.")

class GraphSelectionDialog(QDialog):
    """
    A dialog for selecting which graphs to display in the ``GraphViewer``.

    Inherits
    -------
    ``QDialog``
    """

    __slots__ = ("_available_keys", "_selected_keys")

    def __init__(self, available_keys: list[str], selected_keys: list[str]) -> None:
        super().__init__()
        
        self.setWindowTitle("Select Graphs")
        self._available_keys = available_keys
        self._selected_keys = selected_keys

        self.layout = QGridLayout()
        self.checkboxes: dict[str, QCheckBox] = {}

        for i, key in enumerate(self.available_keys):
            checkbox = QCheckBox(key.replace("_", " ").title())
            checkbox.setChecked(key in self.selected_keys)
            self.checkboxes[key] = checkbox
            self.layout.addWidget(checkbox, i // 2, i % 2)

        self.apply_button = misc.pushbutton_maker("Apply", constants.ICONS.save, self.on_apply_clicked)
        self.layout.addWidget(self.apply_button, (len(self.available_keys) + 1) // 2, 0, 1, 2)

        self.setLayout(self.layout)

    @property
    def selected_keys(self) -> list[str]:
        """Get the currently selected keys from the checkboxes."""

        return self._selected_keys
    
    @selected_keys.setter
    def selected_keys(self, keys: list[str]) -> None:
        """
        Set the selected keys and ensure they are a subset of the available keys.
        Also updates the state of the checkboxes to reflect the new selection.
        
        Parameters
        ----------
        keys
            A list of keys representing the selected graphs to display. Must be a subset of available keys.

        Raises
        ------
        ValueError
            If any of the selected keys are not in the available keys.
        """
        
        if not set(keys).issubset(set(self.available_keys)):
            raise ValueError("Selected keys must be a subset of available keys.")
        
        self._selected_keys = keys
        self.update_checkboxes()
    
    @property
    def available_keys(self) -> list[str]:
        """Get the available keys for graph selection."""

        return self._available_keys
    
    @available_keys.setter
    def available_keys(self, keys: list[str]) -> None:
        """Set the available keys for graph selection and update the checkboxes."""

        if self._available_keys == keys:
            return

        self._available_keys = keys

        for key in keys:
            if key not in self.checkboxes:
                checkbox = QCheckBox(key.replace("_", " ").title())
                checkbox.setChecked(key in self.selected_keys)
                self.checkboxes[key] = checkbox
                row = len(self.checkboxes) // 2
                col = len(self.checkboxes) % 2
                self.layout.addWidget(checkbox, row, col)

    def update_checkboxes(self) -> None:
        """Update the state of the checkboxes based on the selected keys."""

        for key, checkbox in self.checkboxes.items():
            checkbox.setChecked(key in self.selected_keys)

    def on_apply_clicked(self) -> None:
        """Apply the selected graphs and close the dialog."""

        self._selected_keys = [key for key, checkbox in self.checkboxes.items() if checkbox.isChecked()]
        self.accept()
