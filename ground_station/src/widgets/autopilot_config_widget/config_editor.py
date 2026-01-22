import json
from ast import literal_eval
from copy import deepcopy
from pathlib import Path
from typing import Any
from urllib.parse import urljoin

from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)
from requests.exceptions import RequestException
from syntax_highlighters.json import JsonHighlighter
from utils import constants, misc, thread_classes
from widgets.popup_edit import TextEditWindow
from yaml import safe_load


class AutopilotConfigEditor(QWidget):
    """
    A widget for interacting with and editing autopilot parameters.

    Inherits
    --------
    ``QWidget``
    """

    def __init__(self) -> None:
        super().__init__()

        self.timer = misc.copy_qtimer(constants.ONE_SECOND_TIMER)

        self.main_layout = QGridLayout()
        self.setLayout(self.main_layout)

        self.widgets: list[AutopilotParamWidget] = []
        self.config: dict[str, dict[str, Any]] = {}

        # region actions button group
        self.button_group_box = QGroupBox()
        self.button_layout = QHBoxLayout()
        self.button_group_box.setLayout(self.button_layout)

        self.send_all_button = misc.pushbutton_maker(
            "Send All",
            constants.ICONS.upload,
            self.send_all_parameters,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )
        self.pull_all_button = misc.pushbutton_maker(
            "Pull All",
            constants.ICONS.download,
            self.pull_all_parameters,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )
        self.load_from_file_button = misc.pushbutton_maker(
            "Load from File",
            constants.ICONS.hard_drive,
            self.load_parameters_from_file,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )
        self.save_to_file_button = misc.pushbutton_maker(
            "Save to File",
            constants.ICONS.save,
            self.save_parameters_to_file,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )

        self.button_layout.addWidget(self.send_all_button)
        self.button_layout.addWidget(self.pull_all_button)
        self.button_layout.addWidget(self.load_from_file_button)
        self.button_layout.addWidget(self.save_to_file_button)
        # endregion actions button group
        
        try:
            self.config = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_default_autopilot_parameters"],
                    str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                )
            ).json()

        except RequestException as e:
            print(f"[Error] Failed to fetch default autopilot parameters: {e}")
            self.config = {}

        self.params_container = QWidget()
        self.params_layout = QVBoxLayout()
        self.params_layout.setAlignment(Qt.AlignTop)
        self.params_container.setLayout(self.params_layout)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.params_container)

        self.searchbar = QLineEdit()
        self.searchbar.setClearButtonEnabled(True)
        self.searchbar.setPlaceholderText("Search parameters...")
        self.searchbar.textChanged.connect(self.filter_parameters)

        # status label to show search results
        self.status_label = QLabel()
        self.status_label.setStyleSheet("color: #D3D3D3; font-size: 12pt;")
        self.status_label.setAlignment(Qt.AlignCenter)

        self.main_layout.addWidget(self.searchbar, 0, 0)
        self.main_layout.addWidget(self.status_label, 1, 0)
        self.main_layout.addWidget(self.scroll, 2, 0)
        self.main_layout.addWidget(self.button_group_box, 3, 0)

        self.add_parameters()
        self.update_status_label()

        self.hash_fetcher = thread_classes.AutopilotThreadRouter.ActiveHashFetcherThread()
        self.hash_fetcher.response.connect(self.update_hash)
        self.timer.timeout.connect(self.hash_fetcher_starter)
        self.timer.start()

    def send_all_parameters(self) -> None:
        """Send all parameters to the telemetry endpoint."""

        print("[Info] Sending all parameters...")

        existing_data = {}
        for widget in self.widgets:
            if isinstance(widget, AutopilotParamWidget):
                existing_data[widget.name] = widget.value

        try:
            constants.REQ_SESSION.post(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["set_autopilot_parameters"],
                    str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                ),
                json=existing_data,
            )
            print("[Info] All parameters sent successfully.")

        except RequestException as e:
            print(f"[Error] Failed to send all parameters: {e}")

    def pull_all_parameters(self) -> None:
        """Pull all parameters from the telemetry endpoint."""

        print("[Info] Pulling all parameters...")

        try:
            data = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                    str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                )
            ).json()

            if not isinstance(data, dict):
                raise TypeError
            
            if not all(isinstance(key, str) for key in data):
                raise TypeError

            for widget in self.widgets:
                if widget.name in data:
                    widget.value = data[widget.name]
                    if isinstance(widget.modify_element, QLineEdit):
                        widget.modify_element.setText(str(widget.value))
                    elif widget.value_display:
                        widget.value_display.setText(str(widget.value))
                else:
                    print(f"[Warning] {widget.name} not found in pulled data.")

            print("[Info] All parameters pulled successfully.")

        except RequestException as e:
            print(f"[Error] Failed to pull all parameters: {e}")

        except TypeError:
            print(f"[Error] Unexpected data format from telemetry server: {data}. Expected a dictionary of parameters.")

    def load_parameters_from_file(self) -> None:
        """Load parameters from a file."""

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            caption="Load Parameters from File",
            directory=constants.AUTOPILOT_PARAMS_DIR.as_posix(),
            filter="JSON Files (*.json);;All Files (*)",
        )
        if not file_path:
            return

        try:
            with open(file_path, mode="r", encoding="utf-8") as file:
                file_config = json.load(file)

            if not isinstance(file_config, dict):
                raise TypeError("Configuration file must contain a dictionary of parameters.")

            self.config = file_config
            self.add_parameters()
            self.update_status_label()
            print(f"[Info] Loaded parameters from {file_path}.")

        except Exception as e:
            print(f"[Error] Unable to load parameters from file: {e}")

    def save_parameters_to_file(self) -> None:
        """Save parameters to a file."""

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            caption="Load Parameters from File",
            directory=constants.AUTOPILOT_PARAMS_DIR.as_posix(),
            filter="JSON Files (*.json);;All Files (*)"
        )
        if not file_path:
            return

        try:
            with open(file_path, mode="w", encoding="utf-8") as file:
                json.dump(self.config, file, indent=4)
                print(f"[Info] Saved parameters to {file_path}.")

        except Exception as e:
            print(f"[Error] Unable to save parameters to file: {e}")

    def add_parameters(self) -> None:
        """Add all parameters to the layout."""

        for widget in self.widgets:
            widget.deleteLater()

        self.widgets: list[AutopilotParamWidget] = []

        for key in self.config:
            param_config = self.config[key].copy()
            param_config["name"] = key
            try:
                param_widget = AutopilotParamWidget(param_config)
                self.params_layout.addWidget(param_widget)
                self.widgets.append(param_widget)
            
            except Exception as e:
                print(f"Error creating widget for parameter '{key}': {e}")

        # add spacer to push content to top
        if hasattr(self, "spacer"):
            self.params_layout.removeItem(self.spacer)
        self.spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.params_layout.addItem(self.spacer)

    def filter_parameters(self, search_text: str = "") -> None:
        """
        Filter parameters based on search text.

        Parameters
        ----------
        search_text
            The text to filter parameters by. Defaults to an empty string, which shows all parameters.
        """

        search_text = search_text.lower().strip()
        visible_count = 0

        for widget in self.widgets:
            name_match: bool = search_text in widget.name.lower()
            desc_match: bool = search_text in widget.description.lower()

            if search_text and not (name_match or desc_match):
                widget.hide()
            
            else:
                widget.show()
                visible_count += 1

        self.update_status_label(visible_count, search_text)

    def update_status_label(self, visible_count: int | None = None, search_text: str = "") -> None:
        """
        Update the status label with search results.

        Parameters
        ----------
        visible_count
            The number of parameters currently visible after filtering.
            If ``None``, it will be calculated from the number of current widgets.

        search_text
            The text used for filtering parameters. If empty, it indicates that all parameters are shown.
        """

        if visible_count is None:
            visible_count = len(self.widgets)

        if not search_text:
            self.status_label.setText(
                f"Showing all {visible_count} parameters | Showing config: {constants.AUTOPILOT_PARAM_HASH}"
            )
        
        elif visible_count == 0:
            self.status_label.setText(f"No parameters match '{search_text}' | Showing config: {constants.AUTOPILOT_PARAM_HASH}")
        
        else:
            self.status_label.setText(
                f"Showing {visible_count} parameters matching '{search_text}' | Showing config: {constants.AUTOPILOT_PARAM_HASH}"
            )

    def hash_fetcher_starter(self) -> None:
        """Start the hash fetcher thread."""
        
        if not self.hash_fetcher.isRunning():
            self.hash_fetcher.start()

    def update_hash(self, request_result: tuple[str, constants.TelemetryStatus]) -> None:
        """
        Update the autopilot parameter hash based on the thread response.

        Parameters
        ----------
        request_result
            A tuple containing the hash string and the telemetry status.
        """

        hash_string, status = request_result

        if status == constants.TelemetryStatus.SUCCESS:
            constants.AUTOPILOT_PARAM_HASH = hash_string

class AutopilotParamWidget(QFrame):
    """
    A widget for displaying autopilot parameters and interacting with them.

    Parameters
    ----------
    config
        A dictionary containing the parameter configuration. It should include:
        - ``name``: The name of the parameter (str).
        - ``default``: The default value for the parameter.
        - ``description``: A description of the parameter (str).

    Inherits
    --------
    ``QFrame``
    """

    def __init__(self, config: dict) -> None:
        super().__init__()

        # region validate parameter config
        try:
            self.name: str = config["name"]
            self.default_val: Any = config["default"]
            self.type: type = type(self.default_val)
            self.description: str = config["description"]

            assert isinstance(self.name, str), "Parameter name must be a string."
            assert isinstance(self.description, str), "Description must be a string."

        except (KeyError, AssertionError) as e:
            raise ValueError("[Error] Invalid configuration for `AutopilotParamWidget`.") from e
        # endregion validate parameter config

        # region define layouts
        self.current_value = deepcopy(self.default_val)

        self.main_layout = QHBoxLayout(self)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(self.main_layout)

        # for button name and interface to control it
        self.left_layout = QVBoxLayout()
        self.left_layout.setContentsMargins(0, 0, 0, 0)

        # for the send and reset buttons
        self.right_layout = QVBoxLayout()
        self.right_layout.setAlignment(Qt.AlignTop)
        # endregion define layouts

        # region left layout
        self.label = QLabel(f"<b>{self.name}</b>")
        self.label.setToolTip(self.description)
        self.description_label = QLabel(self.description)
        self.description_label.setWordWrap(True)
        self.description_label.setStyleSheet("color: #D3D3D3; font-size: 12pt;")

        if self.type in {list, tuple, set, dict}:
            self.modify_element = QPushButton("Edit")
            self.modify_element.setIcon(constants.ICONS.pencil)
            self.modify_element.clicked.connect(self.edit_grouped_data)
            self.value_display = QLabel(str(self.current_value))
            self.value_display.setWordWrap(True)
            self.value_display.setStyleSheet("font-family: monospace;")

        else:
            self.modify_element = QLineEdit(str(self.current_value))
            self.modify_element.setMinimumWidth(200)
            self.modify_element.editingFinished.connect(self.update_value_from_lineedit)

            # no separate display for simple types
            self.value_display = None

        self.left_layout.addWidget(self.label)
        self.left_layout.addWidget(self.description_label)

        if self.value_display:
            self.left_layout.addWidget(self.value_display)

        self.left_layout.addWidget(self.modify_element)
        # endregion left layout

        # region right layout
        self.send_button = misc.pushbutton_maker(
            "Send",
            constants.ICONS.upload,
            self.send_value,
            max_width=100,
            min_height=30,
            is_clickable=True,
        )
        self.pull_button = misc.pushbutton_maker(
            "Pull",
            constants.ICONS.download,
            self.pull_value,
            max_width=100,
            min_height=30,
            is_clickable=True,
        )
        self.reset_button = misc.pushbutton_maker(
            "Reset",
            constants.ICONS.refresh,
            self.reset_value,
            max_width=100,
            min_height=30,
            is_clickable=False,
        )
        self.right_layout.addWidget(self.send_button)
        self.right_layout.addWidget(self.pull_button)
        self.right_layout.addWidget(self.reset_button)
        # endregion right layout

        self.main_layout.addLayout(self.left_layout, 70)  # 70% width
        self.main_layout.addLayout(self.right_layout, 30)  # 30% width

        self.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.setLineWidth(1)

    def send_value(self) -> None:
        """Send the current value of the parameter to the telemetry endpoint."""

        print(f"[Info] Sending value for {self.name}: {self.current_value}")
        try:
            existing_data = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                    str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                )
            ).json()

        except RequestException as e:
            print(f"[Error] Failed to fetch existing autopilot parameters. Cannot send {self.name}: {e}")
            return

        if isinstance(existing_data, dict):
            if existing_data.get(self.name, None) is None:
                print(f"[Warning] {self.name} not found in existing parameters. Adding it.")
            
            existing_data[self.name] = self.current_value

            try:
                constants.REQ_SESSION.post(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["set_autopilot_parameters"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    ),
                    json=existing_data,
                )
                print(f"[Info] Successfully sent {self.name} with value {self.current_value}.")

            except RequestException as e:
                print(f"[Error] Failed to send {self.name} with value {self.current_value}: {e}")
                return

        else:
            print(f"[Error] Unexpected data format from telemetry server: {existing_data}. Expected a dictionary of parameters.")
            return

        self.reset_button.setEnabled(True)
        self.send_button.setEnabled(False)
        self.pull_button.setEnabled(False)

    def pull_value(self) -> None:
        """Pull the current value of the parameter from the telemetry endpoint."""

        try:
            data = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                    str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                )
            ).json()

            if self.name in data:
                self.current_value = data[self.name]
                if isinstance(self.modify_element, QLineEdit):
                    self.modify_element.setText(str(self.current_value))

                elif self.value_display:
                    self.value_display.setText(str(self.current_value))
                print(f"[Info] Pulled {self.name} with value {self.current_value}.")

                with open(Path(constants._autopilot_param_editor_dir / "params_temp.json"), mode="r", encoding="utf-8") as file:
                    temp_params = json.load(file)

                temp_params[self.name] = {"current": self.current_value, "description": self.description}

                with open(Path(constants._autopilot_param_editor_dir / "params_temp.json"), mode="w", encoding="utf-8") as file:
                    json.dump(temp_params, file, indent=4)

            else:
                print(f"[Warning] {self.name} not found in pulled data.")

        except RequestException as e:
            print(f"[Error] Failed to pull value for {self.name}: {e}")

        self.reset_button.setEnabled(True)
        self.send_button.setEnabled(False)
        self.pull_button.setEnabled(False)

    def reset_value(self) -> None:
        """Reset the value of the parameter to its default value."""

        self.current_value = deepcopy(self.default_val)
        if isinstance(self.modify_element, QLineEdit):
            self.modify_element.setText(str(self.current_value))

        elif self.value_display:
            self.value_display.setText(str(self.current_value))
            print(f"[Info] {self.name} reset to default value: {self.current_value}.")

        with open(Path(constants._autopilot_param_editor_dir / "params_temp.json"), mode="r", encoding="utf-8") as file:
            temp_params = json.load(file)
        
        temp_params[self.name] = {"current": self.current_value, "description": self.description}

        with open(Path(constants._autopilot_param_editor_dir / "params_temp.json"), mode="w", encoding="utf-8") as file:
            json.dump(temp_params, file, indent=4)

        self.reset_button.setEnabled(False)
        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)

    def update_value_from_lineedit(self) -> None:
        """
        Update value from ``QLineEdit`` input.
        
        Raises
        ------
        TypeError
            If the edited data is not of the expected type.
        """

        try:
            edited_data = literal_eval(self.modify_element.text())

            if not isinstance(edited_data, self.type):
                if self.type is bool and (
                    isinstance(edited_data, (int, str)) and edited_data in {0, 1, "true", "false"}
                ):
                    edited_data = bool(edited_data)

                elif self.type is float and isinstance(edited_data, int):
                    edited_data = float(edited_data)

                elif self.type is int and isinstance(edited_data, float) and edited_data.is_integer():
                    edited_data = int(edited_data)

                elif not isinstance(edited_data, self.type):
                    raise TypeError(f"Edited data must be of type {self.type.__name__}, but got {type(edited_data).__name__}.")

            with open(Path(constants._autopilot_param_editor_dir / "params_temp.json"), mode="r", encoding="utf-8") as file:
                temp_params = json.load(file)

            temp_params[self.name] = {"current": edited_data, "default": self.default_val, "description": self.description}

            with open(Path(constants._autopilot_param_editor_dir / "params_temp.json"), mode="w", encoding="utf-8") as file:
                json.dump(temp_params, file, indent=4)

        except TypeError:
            print(f"[Error] Invalid value for {self.name}. Resetting to previous value.")
            self.modify_element.setText(str(self.current_value))
            return

        except Exception as e:
            print(f"[Error] Failed to update value for {self.name}: {e}")
            return

        self.current_value = edited_data
        self.modify_element.setText(str(self.current_value))

        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)
        self.reset_button.setEnabled(True)

    def edit_grouped_data(self) -> None:
        """Open a text editor for editing a sequence of values."""

        try:
            initial_text = json.dumps(self.current_value, indent=2)
            self.text_edit_window = TextEditWindow(highlighter=JsonHighlighter, initial_text=initial_text)
            self.text_edit_window.setWindowTitle(f"Edit {self.name}")
            self.text_edit_window.user_text_emitter.connect(self.edit_grouped_data_callback)
            self.text_edit_window.show()

        except Exception as e:
            print(f"[Error] Failed to open text edit window for {self.name}: {e}")

    def edit_grouped_data_callback(self, text: str) -> None:
        """
        Callback function for the ``edit_grouped_data`` function.

        Parameters
        ----------
        text
            The text entered by the user in the text editor.

        Raises
        ------
        TypeError
            If the edited data is not of the expected type.
        """

        try:
            edited_data = safe_load(text)

            if edited_data == self.current_value:
                return

            if not isinstance(edited_data, self.type):
                raise TypeError(f"Edited data must be of type {self.type.__name__}, but got {type(edited_data).__name__}.")

        except TypeError:
            print(f"[Error] Invalid value for {self.name}. Resetting to previous value.")
            self.value_display.setText(str(self.current_value))
            return

        self.current_value = edited_data
        self.value_display.setText(str(self.current_value))
        print(f"[Info] {self.name} updated to {self.current_value}.")

        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)
        self.reset_button.setEnabled(True)
