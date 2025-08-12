import constants
from syntax_highlighters.json import JsonHighlighter
from widgets.popup_edit import TextEditWindow
from copy import deepcopy
from yaml import safe_load
import requests
import json
from typing import Any
from pathlib import PurePath
from jsonc_parser.parser import JsoncParser
from qtpy.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QSpacerItem,
    QSizePolicy,
    QScrollArea,
    QPushButton,
    QFrame,
    QGroupBox,
    QFileDialog,
)
from qtpy.QtCore import Qt


class AutopilotParamEditor(QWidget):
    """
    A widget for interacting with and editing autopilot parameters.

    Inherits
    --------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.main_layout = QGridLayout()
        self.setLayout(self.main_layout)

        self.config: dict[str, dict[str, Any]] = {}
        self.widgets: list[AutopilotParamWidget] = []

        # region actions button group
        self.button_group_box = QGroupBox()
        self.button_layout = QHBoxLayout()
        self.button_group_box.setLayout(self.button_layout)

        self.send_all_button = constants.pushbutton_maker(
            "Send All",
            constants.ICONS.upload,
            self.send_all_parameters,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )
        self.pull_all_button = constants.pushbutton_maker(
            "Pull All",
            constants.ICONS.download,
            self.pull_all_parameters,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )
        self.load_from_file_button = constants.pushbutton_maker(
            "Load from File",
            constants.ICONS.hard_drive,
            self.load_parameters_from_file,
            max_width=200,
            min_height=30,
            is_clickable=True,
        )
        self.save_to_file_button = constants.pushbutton_maker(
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
            self.config = JsoncParser.parse_file(constants.AUTO_PILOT_PARAMS_DIR / "params_default.jsonc")
            print(
                f"[Info] Loaded {len(self.config)} parameters from `{constants.AUTO_PILOT_PARAMS_DIR / 'params_default.jsonc'}`."
            )
        except Exception:
            print("[Error] Please ensure the file exists in the `app_data/autopilot_params` directory.")

        self.params_container = QWidget()
        self.params_layout = QVBoxLayout()
        self.params_layout.setAlignment(Qt.AlignTop)
        self.params_container.setLayout(self.params_layout)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.params_container)

        self.searchbar = QLineEdit()
        self.searchbar.setPlaceholderText("Search parameters...")
        self.searchbar.textChanged.connect(self.filter_parameters)

        self.searchbar.setClearButtonEnabled(True)

        # Status label to show search results
        self.status_label = QLabel()
        self.status_label.setStyleSheet("color: #D3D3D3; font-size: 12pt;")
        self.status_label.setAlignment(Qt.AlignCenter)

        self.main_layout.addWidget(self.searchbar, 0, 0)
        self.main_layout.addWidget(self.status_label, 1, 0)
        self.main_layout.addWidget(self.scroll, 2, 0)
        self.main_layout.addWidget(self.button_group_box, 3, 0)

        self.add_parameters()
        self.update_status_label()

    def send_all_parameters(self) -> None:
        """Send all parameters to the telemetry endpoint."""

        print("[Info] Sending all parameters...")
        existing_data = {}
        for widget in self.widgets:
            if isinstance(widget, AutopilotParamWidget):
                existing_data[widget.name] = widget.value

        try:
            response = constants.REQ_SESSION.post(
                constants.TELEMETRY_SERVER_ENDPOINTS["set_autopilot_parameters"],
                json={"value": existing_data},
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            )
            response.raise_for_status()
            print("[Info] All parameters sent successfully.")
        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to send all parameters: {e}")

    def pull_all_parameters(self) -> None:
        """Pull all parameters from the telemetry endpoint."""

        print("[Info] Pulling all parameters...")
        try:
            response = constants.REQ_SESSION.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            )
            response.raise_for_status()
            data = response.json()

            for widget in self.widgets:
                if isinstance(widget, AutopilotParamWidget):
                    if widget.name in data:
                        widget.value = data[widget.name]
                        if isinstance(widget.modify_element, QLineEdit):
                            widget.modify_element.setText(str(widget.value))
                        elif widget.value_display:
                            widget.value_display.setText(str(widget.value))
                    else:
                        print(f"[Warning] {widget.name} not found in pulled data.")

            print("[Info] All parameters pulled successfully.")
        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to pull all parameters: {e}")

    def load_parameters_from_file(self) -> None:
        """Load parameters from a file."""

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Parameters from File",
            "",
            "JSONC Files (*.jsonc);;JSON Files (*.json);;All Files (*)",
        )
        if not file_path:
            return

        try:
            data = JsoncParser.parse_file(PurePath(file_path))
            self.config = data
            self.add_parameters()
            self.update_status_label()
            print(f"[Info] Loaded parameters from {file_path}.")
        except Exception as e:
            print(f"[Error] Unable to read from file: {e}")

    def save_parameters_to_file(self) -> None:
        """Save parameters to a file."""

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Parameters to File", "", "JSON Files (*.json);;All Files (*)"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w") as file:
                json.dump(self.config, file, indent=4)
                print(f"[Info] Saved parameters to {file_path}.")
        except Exception as e:
            print(f"[Error] Unable to save parameters to file: {e}")

    def add_parameters(self) -> None:
        """Add all parameters to the layout."""

        # Clear existing widgets
        for widget in self.widgets:
            widget.deleteLater()
        self.widgets = []

        for key in self.config:
            # Create config with name
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
            name_match = search_text in widget.name.lower()
            desc_match = search_text in widget.description.lower()

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
            If `None`, it will be calculated from the number of current widgets.

        search_text
            The text used for filtering parameters. If empty, it indicates that all parameters are shown.
        """

        if visible_count is None:
            visible_count = len(self.widgets)

        if not search_text:
            self.status_label.setText(f"Showing all {visible_count} parameters")
        elif visible_count == 0:
            self.status_label.setText(f"No parameters match '{search_text}'")
        else:
            self.status_label.setText(f"Showing {visible_count} parameters matching '{search_text}'")


class AutopilotParamWidget(QFrame):
    """
    A widget for displaying autopilot parameters and interacting with them.

    Parameters
    ----------
    config
        A dictionary containing the parameter configuration. It should include:
        - `name`: The name of the parameter (str).
        - `type`: The type of the parameter (str, one of "bool", "int", "float", "str", "list", "dict", "tuple", "set").
        - `default`: The default value for the parameter, which should match the specified type.
        - `description`: A description of the parameter (str).

    Inherits
    --------
    `QFrame`
    """

    def __init__(self, config: dict) -> None:
        super().__init__()

        type_map = {
            "bool": bool,
            "int": int,
            "float": float,
            "str": str,
            "list": list,
            "dict": dict,
            "tuple": tuple,
            "set": set,
        }

        # region validate parameter config
        try:
            self.name: str = config["name"]
            self.type: type = type_map[config["type"]]
            self.default_val = config["default"]
            self.description: str = config["description"]

            assert isinstance(self.name, str), "Parameter name must be a string."
            assert isinstance(self.default_val, self.type), f"Default value must be of type {self.type.__name__}."
            assert isinstance(self.description, str), "Description must be a string."

        except (KeyError, IndexError):
            print(
                "Invalid configuration for `AutopilotParamWidget`. See `src/widgets/autopilot_param_editor/editor_config.jsonc`."
            )
        # endregion validate parameter config

        # region define layouts
        self.value = deepcopy(self.default_val)

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

        if self.type in (list, dict, tuple, set):
            self.modify_element = QPushButton("Edit")
            self.modify_element.setIcon(constants.ICONS.pencil)
            self.modify_element.clicked.connect(self.edit_sequence_data)
            self.value_display = QLabel(str(self.value))
            self.value_display.setWordWrap(True)
            self.value_display.setStyleSheet("font-family: monospace;")
        else:
            self.modify_element = QLineEdit(str(self.value))
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
        self.send_button = constants.pushbutton_maker(
            "Send",
            constants.ICONS.upload,
            self.send_value,
            max_width=100,
            min_height=30,
            is_clickable=True,
        )
        self.pull_button = constants.pushbutton_maker(
            "Pull",
            constants.ICONS.download,
            self.pull_value,
            max_width=100,
            min_height=30,
            is_clickable=True,
        )
        self.reset_button = constants.pushbutton_maker(
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

        print(f"[Info] Sending value for {self.name}: {self.value}")
        try:
            existing_data = constants.REQ_SESSION.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to fetch existing autopilot parameters. Cannot send {self.name}: {e}")
            return

        if isinstance(existing_data, dict):
            if existing_data.get(self.name, None) is not None:
                existing_data[self.name] = self.value
            else:
                print(f"[Warning] {self.name} not found in existing parameters. Adding it.")
                existing_data[self.name] = self.value

            try:
                response = constants.REQ_SESSION.post(
                    constants.TELEMETRY_SERVER_ENDPOINTS["set_autopilot_parameters"],
                    json={"value": existing_data},
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                response.raise_for_status()
                print(f"[Info] Successfully sent {self.name} with value {self.value}.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to send {self.name} with value {self.value}: {e}")
                return

        else:
            print(
                f"[Error] Unexpected data format from telemetry server: {existing_data}. "
                "Expected a dictionary of parameters."
            )
            return

        self.reset_button.setEnabled(True)
        self.send_button.setEnabled(False)
        self.pull_button.setEnabled(False)

    def pull_value(self) -> None:
        """Pull the current value of the parameter from the telemetry endpoint."""

        try:
            response = constants.REQ_SESSION.get(constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"])
            response.raise_for_status()
            data = response.json()

            if self.name in data:
                self.value = data[self.name]
                if isinstance(self.modify_element, QLineEdit):
                    self.modify_element.setText(str(self.value))

                elif self.value_display:
                    self.value_display.setText(str(self.value))
                print(f"[Info] Pulled {self.name} with value {self.value}.")

            else:
                print(f"[Warning] {self.name} not found in pulled data.")

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to pull value for {self.name}: {e}")

        self.reset_button.setEnabled(True)
        self.send_button.setEnabled(False)
        self.pull_button.setEnabled(False)

    def reset_value(self) -> None:
        """Reset the value of the parameter to its default value."""

        self.value = deepcopy(self.default_val)
        if isinstance(self.modify_element, QLineEdit):
            self.modify_element.setText(str(self.value))

        elif self.value_display:
            self.value_display.setText(str(self.value))
            print(f"[Info] {self.name} reset to default value: {self.value}.")

        self.reset_button.setEnabled(False)
        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)

    def update_value_from_lineedit(self) -> None:
        """Update value from `QLineEdit` input."""

        try:
            edited_data = safe_load(self.modify_element.text())

            if self.type is bool:
                if isinstance(edited_data, (int, float)):
                    if edited_data in (0, 1):
                        edited_data = bool(edited_data)

                elif isinstance(edited_data, bool):
                    edited_data = bool(edited_data)

            if self.type is float and isinstance(edited_data, int):
                edited_data = float(edited_data)

            if not isinstance(edited_data, self.type):
                raise TypeError(
                    f"Edited data must be of type {self.type.__name__}, but got {type(edited_data).__name__}."
                )

            with open(
                PurePath(constants._autopilot_param_editor_dir / "params_temp.json"),
            ) as file:
                temp_params = json.load(file)

            temp_params[self.name] = {"type": self.type.__name__, "value": edited_data}

            with open(
                PurePath(constants._autopilot_param_editor_dir / "params_temp.json"),
                "w",
            ) as file:
                json.dump(temp_params, file, indent=4)

        except TypeError:
            print(f"[Error] Invalid value for {self.name}. Resetting to previous value.")
            self.modify_element.setText(str(self.value))
            return

        except Exception as e:
            print(f"[Error] Failed to update value for {self.name}: {e}")
            return

        self.value = edited_data
        self.modify_element.setText(str(self.value))

        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)
        self.reset_button.setEnabled(True)

    def edit_sequence_data(self) -> None:
        """Open a text editor for editing a sequence of values."""

        try:
            initial_text = json.dumps(self.value, indent=2)
            self.text_edit_window = TextEditWindow(highlighter=JsonHighlighter, initial_text=initial_text)
            self.text_edit_window.setWindowTitle(f"Edit {self.name}")
            self.text_edit_window.user_text_emitter.connect(self.edit_sequence_data_callback)
            self.text_edit_window.show()

        except Exception as e:
            print(f"[Error] Failed to open text edit window for {self.name}: {e}")

    def edit_sequence_data_callback(self, text: str) -> None:
        """
        Callback function for the `edit_sequence_data` function.

        Parameters
        ----------
        text
            The text entered by the user in the text editor.
        """

        try:
            edited_data = safe_load(text)

            if edited_data == self.value:
                return

            if not isinstance(edited_data, self.type):
                raise TypeError(
                    f"Edited data must be of type {self.type.__name__}, but got {type(edited_data).__name__}."
                )

        except TypeError:
            print(f"[Error] Invalid value for {self.name}. Resetting to previous value.")
            self.value_display.setText(str(self.value))
            return

        self.value = edited_data
        self.value_display.setText(str(self.value))
        print(f"[Info] {self.name} updated to {self.value}.")

        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)
        self.reset_button.setEnabled(True)
