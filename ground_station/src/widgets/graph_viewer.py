import time
import requests
import numpy as np
import numpy.typing as npt
from urllib.parse import urljoin
from typing import Any
from utils import constants, thread_classes, misc

from qtpy.QtCore import Qt
from qtpy.QtWidgets import QWidget, QGridLayout, QCheckBox, QDialog
import pyqtgraph as pg


class GraphViewer(QWidget):
    """
    A widget for monitoring data in real-time using graphs.

    Inherits
    -------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.timer = misc.copy_qtimer(constants.TEN_MS_TIMER)
        self.plots: list[pg.PlotItem] = []
        self.time_stopped: float = 0
        self.time_started: float = 0
        self.important_keys: list[str] = ["speed", "distance_to_next_waypoint", "bearing", "heading", "true_wind_speed"]
        self.available_keys: list[str] = []

        self.history_length: int = 500
        self.x_axis: list[float] = []
        self.data: dict[str, list[npt.NDArray[np.float64]]] = {}

        self.main_layout = QGridLayout()

        self.graph_layout_widget = pg.GraphicsLayoutWidget()
        self.main_layout.addWidget(self.graph_layout_widget, 0, 0, 1, 2)

        self.open_graphs_dialog_button = misc.pushbutton_maker("Select Graphs", constants.ICONS.cog, self.select_graphs)
        self.unpause_button = misc.pushbutton_maker("Unpause Graphs", constants.ICONS.play, self.unpause_timer)
        self.pause_button = misc.pushbutton_maker(
            "Pause Graphs", constants.ICONS.pause, self.pause_timer, is_clickable=False
        )
        self.main_layout.addWidget(self.unpause_button, 1, 0)
        self.main_layout.addWidget(self.pause_button, 1, 1)
        self.main_layout.addWidget(self.open_graphs_dialog_button, 2, 0, 1, 2)
        self.setLayout(self.main_layout)

        self.telemetry_handler = thread_classes.TelemetryUpdater()
        self.timer.timeout.connect(self.update_graph_start)
        self.telemetry_handler.boat_data_fetched.connect(self.update_graph)

    def update_graph(self, boat_data: dict[str, Any]) -> None:
        """Update the graphs with new telemetry data.

        Parameters
        ----------
        boat_data
            A dictionary containing the latest telemetry data.
        """

        try:
            self.available_keys = {k: v for k, v in boat_data.items() if isinstance(v, (int, float))}
            boat_data = {k: v for k, v in boat_data.items() if k in self.important_keys}
            current_time = time.time()

            if len(self.x_axis) < self.history_length:
                self.x_axis.append(current_time)
                for key in boat_data.keys():
                    if key not in self.data:
                        self.data[key] = [np.array([boat_data[key]], dtype=np.float64)]

                    else:
                        self.data[key].append(np.array([boat_data[key]], dtype=np.float64))

            else:
                self.x_axis.pop(0)
                self.x_axis.append(current_time)
                for key in boat_data.keys():
                    if key not in self.data:
                        self.data[key] = [np.array([boat_data[key]], dtype=np.float64)]

                    else:
                        self.data[key].pop(0)
                        self.data[key].append(np.array([boat_data[key]], dtype=np.float64))

            if not self.plots:
                num_keys = len(self.important_keys)
                last_is_odd = num_keys % 2 == 1

                for i, key in enumerate(self.data.keys()):
                    # make last plot span 2 columns if number of plots is odd
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

                    # set equal preferred widths for all plots
                    if col_span == 1:
                        plot_item.setPreferredWidth(400)

                    plot_item.setMouseEnabled(x=True, y=True)
                    plot_item.setLabel("bottom", "Time", "s")
                    plot_item.showGrid(x=True, y=True)

                    self.plots.append(plot_item)
                    curve = plot_item.plot(pen=pg.mkPen(color=(255, 0, 0)))
                    curve.setData(self.x_axis, np.concatenate(self.data[key]))

            else:
                for i, key in enumerate(self.data.keys()):
                    curve = self.plots[i].listDataItems()[0]
                    curve.setData(self.x_axis, np.concatenate(self.data[key]))

        except Exception as e:
            print(f"[Error] Failed to update graphs: {e}")

    def select_graphs(self) -> None:
        """Open a dialog to select which graphs to display."""

        if not self.available_keys:
            print("[Warning] No available telemetry data to select graphs from. Pulling keys from telemetry server...")
            try:
                response = constants.REQ_SESSION.get(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["get_boat_status"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    ),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()

                if not isinstance(response, dict):
                    raise TypeError

                self.available_keys = {
                    k: v for k, v in response.items() if isinstance(v, (int, float)) and k != "current_waypoint_index"
                }
                print("[Info] Successfully pulled available telemetry keys from server.")

            except requests.exceptions.RequestException:
                print("[Error] Failed to connect to telemetry server to fetch available keys.")
                return

            except TypeError:
                print(f"[Error] Unexpected response format from telemetry server: {response}")
                return

        self.graph_dialog = GraphSelectionDialog(available_keys=self.available_keys, selected_keys=self.important_keys)
        self.graph_dialog.setWindowModality(Qt.ApplicationModal)
        self.graph_dialog.show()
        self.graph_dialog.apply_button.clicked.connect(
            lambda: self.apply_graph_selection(self.graph_dialog.selected_keys)
        )

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

    def pause_timer(self) -> None:
        """Pause the timer which fetches data for the plots."""

        self.time_stopped = time.time()
        self.timer.stop()

        self.x_axis.clear()
        self.data.clear()
        self.plots.clear()
        self.graph_layout_widget.clear()

        self.pause_button.setDisabled(True)
        self.unpause_button.setDisabled(False)
        print("[Info] Graph widget paused.")

    def unpause_timer(self) -> None:
        """Unpause the timer which fetches data for the plots."""

        self.time_started = time.time()
        elapsed_time = self.time_started - self.time_stopped
        self.timer.start()
        self.pause_button.setDisabled(False)
        self.unpause_button.setDisabled(True)

        print(f"[Info] Graph widget unpaused after being paused for {elapsed_time:.2f} seconds.")

    def update_graph_start(self) -> None:
        """Start the telemetry data fetching thread."""

        if constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED:
            self.timer.stop()
            self.x_axis.clear()
            self.data.clear()
            self.plots.clear()
            self.graph_layout_widget.clear()
            self.pause_button.setDisabled(True)
            self.unpause_button.setDisabled(False)

        if not self.telemetry_handler.isRunning():
            self.telemetry_handler.start()


class GraphSelectionDialog(QDialog):
    """
    A dialog for selecting which graphs to display in the GraphViewer.

    Inherits
    -------
    `QDialog`
    """

    def __init__(self, available_keys: list[str], selected_keys: list[str]) -> None:
        super().__init__()
        self.setWindowTitle("Select Graphs")
        self.available_keys = available_keys
        self.selected_keys = selected_keys

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

    def on_apply_clicked(self) -> None:
        """Apply the selected graphs and close the dialog."""

        self.selected_keys = [key for key, checkbox in self.checkboxes.items() if checkbox.isChecked()]
        self.accept()
