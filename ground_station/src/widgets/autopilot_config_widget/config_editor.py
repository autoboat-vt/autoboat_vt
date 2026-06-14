import hashlib
import json
from ast import literal_eval
from copy import deepcopy
from pathlib import Path
from requests.exceptions import RequestException
from typing import Any
from urllib.parse import urljoin

from qtpy.QtCore import Qt, Signal, Slot
from qtpy.QtWidgets import (
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)

from utils import TextEditWindow, constants, misc, syntax_highlighters
from utils.dialog_templates import MessageBoxButton, show_message_box


class AutopilotConfigEditor(QWidget):
    """
    A widget for interacting with and editing autopilot parameters.

    Parameters
    ----------
    refresh_signal: ``Signal``
        Signal emitted when the autopilot configuration needs to be refreshed. This
        typically occurs after switching telemetry server instances.

    Inherits
    --------
    ``QWidget``
    """

    def __init__(self, refresh_signal: Signal) -> None:
        super().__init__()

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

        self.show_load_warning: bool = True
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
                    misc.get_route("get_default_autopilot_parameters"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                )
            ).json()

            constants.SM.write("current_autopilot_parameters", self.config)
            constants.SM.write("local_autopilot_param_hash", constants.SM.read_str("remote_autopilot_param_hash"))
            print("[Info] Fetched default autopilot parameters successfully.")

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

        self.refresh_signal = refresh_signal
        self.refresh_signal.connect(self.refresh_config)

    @Slot()
    def send_all_parameters(self) -> None:
        """
        Send all parameters to the telemetry server.

        This method handles three scenarios when sending parameters:
        1. If the telemetry server does not have default parameters set (indicated by an empty remote hash),
        the user will be prompted to set the current parameters as the default configuration on the telemetry server.
        
        2. If the remote hash does not match the local hash, it indicates a potential mismatch between the
        local and remote configurations. The user will be prompted to set the current local parameters as the default
        configuration on the telemetry server to avoid unintended consequences of sending an update with mismatched fields.
        
        3. If the remote hash matches the local hash, the parameters will be sent as an update since the
        available fields are known to match and there won't be any unexpected fields.
        """

        print("[Info] Sending all parameters...")

        try:
            tmp_parameters = deepcopy(self.config)
            for parameter in tmp_parameters.values():
                if "current" in parameter:
                    parameter["default"] = parameter.pop("current")
            
            remote_hash = constants.SM.read_str("remote_autopilot_param_hash")

            if remote_hash == "":
                # region get user input
                response = show_message_box(
                    title="Set Default Parameters",
                    message=(
                        "Default parameters are not set on the telemetry server. Do you want to set the current "
                        "parameters as the default configuration on the telemetry server?"
                    ),
                    icon=constants.ICONS.warning,
                    buttons=[QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No],
                )

                if response == QMessageBox.StandardButton.No:
                    print("[Info] Sending all parameters cancelled by user.")
                    return
                
                print("[Info] Setting current parameters as default configuration on telemetry server.")
                response = constants.REQ_SESSION.post(
                    urljoin(
                        misc.get_route("set_default_autopilot_parameters"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    ),
                    json=json.dumps(tmp_parameters, indent=None)
                )
                # endregion get user input
                # region handle response
                status_message = response.text.strip().replace('"', "")

                if response.status_code == 200:
                    print("[Info] Current parameters set as default successfully.")

                elif status_message == "Configuration hash already exists.":
                    local_ = hashlib.sha256(
                        json.dumps(
                            tmp_parameters, sort_keys=True, separators=(",", ":")
                        ).encode(encoding="utf-8")
                    ).hexdigest()

                    response = constants.REQ_SESSION.post(
                        urljoin(
                            misc.get_route("set_default_from_hash"),
                            str(constants.SM.read_int("telemetry_server_instance_id")) + "/" + local_,
                        )
                    )

                    if response.status_code == 200:
                        print(
                            f"[Info] Default parameters set successfully from existing config with "
                            f"matching hash {local_}."
                        )

                    else:
                        print(
                            f"[Warning] Failed to set default parameters from existing config with "
                            f"matching hash {local_}; status {response.status_code}: "
                            f"{status_message}"
                        )

                else:
                    raise RequestException(status_message)
                
                # endregion handle response

            elif remote_hash != constants.SM.read_str("local_autopilot_param_hash"):
                # region get user input
                response = show_message_box(
                    title="Hash Mismatch",
                    message=(
                        "The remote autopilot parameters have changed since the last time they were fetched, and the "
                        "hash of the current local configuration does not match the hash of the remote configuration. "
                        "This may indicate that there are parameters in the current local configuration that are not "
                        "present in the remote configuration, which could lead to unintended consequences if sent as an update. "
                        "Do you want to set the current local parameters as the default configuration on the telemetry server?"
                    ),
                    icon=constants.ICONS.warning,
                    buttons=[QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No],
                )

                if response == QMessageBox.StandardButton.No:
                    print("[Info] Sending all parameters cancelled by user due to hash mismatch.")
                    return

                print(
                    "[Info] Setting current local parameters as default configuration on telemetry "
                    "server due to hash mismatch."
                )
                response = constants.REQ_SESSION.post(
                    urljoin(
                        misc.get_route("set_default_autopilot_parameters"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    ),
                    json=json.dumps(tmp_parameters, indent=None)
                )
                # endregion get user input
                # region handle response
                status_message = response.text.strip().replace('"', "")

                if response.status_code == 200:
                    print("[Info] Current parameters set as default successfully.")

                elif status_message == "Configuration hash already exists.":
                    hash_from_local_config = hashlib.sha256(
                        json.dumps(tmp_parameters, sort_keys=True, separators=(",", ":")).encode(encoding="utf-8")
                    ).hexdigest()

                    response = constants.REQ_SESSION.post(
                        urljoin(
                            misc.get_route("set_default_from_hash"),
                            str(constants.SM.read_int("telemetry_server_instance_id")) + "/" + hash_from_local_config,
                        )
                    )

                    if response.status_code == 200:
                        print(
                            f"[Info] Default parameters set successfully from existing config with "
                            f"matching hash {hash_from_local_config}."
                        )

                    else:
                        print(
                            f"[Warning] Failed to set default parameters from existing config with "
                            f"matching hash {hash_from_local_config}; status {response.status_code}: "
                            f"{status_message}"
                        )

                else:
                    raise RequestException(status_message)
                
                # endregion handle response

            else:
                print("[Info] Remote hash matches local hash. Sending parameters as an update.")
                response = constants.REQ_SESSION.post(
                    urljoin(
                        misc.get_route("set_autopilot_parameters"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    ),
                    json=json.dumps(tmp_parameters, indent=None)
                )
                status_message = response.text.strip().replace('"', "")

                if response.status_code == 200:
                    print("[Info] All parameters sent successfully.")
                    constants.SM.write("current_autopilot_parameters", tmp_parameters)

                else:
                    raise RequestException(status_message)

        except RequestException as e:
            response = show_message_box(
                title="Failed to Send Parameters",
                message=str(e),
                icon=constants.ICONS.warning
            )

    @Slot()
    def pull_all_parameters(self) -> None:
        """
        Pull all parameters from the telemetry server.

        This method handles three scenarios when pulling parameters:
        1. If the telemetry server does not have default parameters set (indicated by an empty remote hash),
        a warning will be printed and the pull operation will be aborted since there are no parameters to pull.

        2. If the remote parameters are missing any parameters that are currently in the local configuration,
        it indicates a potential mismatch between the local and remote configurations. The user will be prompted
        to replace the existing local configuration with the pulled data to avoid unintended consequences.

        3. If the remote parameters include all parameters that are currently in the local configuration, the local
        configuration will be updated with the pulled values since there won't be any unexpected fields.
        """

        print("[Info] Pulling all parameters...")

        rememeber_choice: bool = False
        response = QMessageBox.StandardButton.No

        try:
            remote_hash = constants.SM.read_str("remote_autopilot_param_hash")
            if remote_hash == "":
                print("[Warning] Default parameters are not set on the telemetry server. Aborting pull operation.")
                return

            remote_parameters = constants.REQ_SESSION.get(
                urljoin(
                    misc.get_route("get_autopilot_parameters"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                )
            ).json()
            if not isinstance(remote_parameters, dict):
                raise TypeError(
                    "Unexpected data format for remote parameters: "
                    f"expected a dictionary, but got {type(remote_parameters).__name__}."
                )

            temp_params: dict[str, dict[str, Any]] = deepcopy(self.config)
            for widget in self.widgets:
                raw_value = remote_parameters.get(widget.name)
                
                if isinstance(raw_value, dict):
                    if "current" in raw_value:
                        new_value = raw_value["current"]
                    elif "default" in raw_value:
                        new_value = raw_value["default"]
                    else:
                        raise TypeError(
                            f"Unexpected data format for parameter '{widget.name}': "
                            f"expected a dictionary with 'current' or 'default' keys, but got {raw_value}."
                        )
                    
                    widget.current_value = new_value
                    if isinstance(widget.modify_element, QLineEdit):
                        widget.modify_element.setText(str(widget.current_value))

                    elif widget.value_display:
                        widget.value_display.setText(str(widget.current_value))

                    temp_params[widget.name] = {
                        "current": new_value,
                        "default": widget.default_val,
                        "description": widget.description,
                    }

                elif raw_value is not None:
                    raise TypeError(
                        f"Unexpected data format for parameter '{widget.name}': "
                        f"expected a dictionary with 'current' or 'default' keys, "
                        f"but got {type(raw_value).__name__}."
                    )

                else:
                    print(f"[Warning] {widget.name} not found in pulled data.")
                    if not rememeber_choice and response == QMessageBox.StandardButton.No:
                        response, rememeber_choice = show_message_box(
                            title="Parameter Not Found",
                            message=(
                                f"The parameter '{widget.name}' was not found in the pulled data. "
                                "Do you want to replace the existing config with the pulled data?"
                            ),
                            icon=constants.ICONS.warning,
                            buttons=[QMessageBox.Yes, QMessageBox.No],
                            remember_choice_option=True,
                        )

                        if response == QMessageBox.StandardButton.Yes:
                            print("[Info] Replacing existing config with pulled data.")
                            
                            default_params = constants.REQ_SESSION.get(
                                urljoin(
                                    misc.get_route("get_default_autopilot_parameters"),
                                    str(constants.SM.read_int("telemetry_server_instance_id")),
                                )
                            ).json()

                            if not isinstance(default_params, dict):
                                raise TypeError(
                                    "Unexpected data format for default parameters: "
                                    f"expected a dictionary, but got {type(default_params).__name__}."
                                )
                            
                            temp_params = default_params
                            self.add_parameters()
                            self.update_status_label()

            print("[Info] All parameters pulled successfully.")
            constants.SM.write("current_autopilot_parameters", temp_params)

        except RequestException as e:
            print(f"[Error] Failed to pull all parameters: {e}")

        except TypeError as e:
            print(f"[Error] {e}")

    def clear_local_config(self) -> None:
        """Clear the local configuration and reset the UI."""

        self.config = {}
        constants.SM.write("current_autopilot_parameters", self.config)
        constants.SM.write("local_autopilot_param_hash", "")
        self.add_parameters()
        self.update_status_label()
        print("[Info] Local configuration cleared.")

    def clear_remote_config(self) -> None:
        """Clear the remote configuration on the telemetry server."""

        try:
            response = constants.REQ_SESSION.post(
                urljoin(
                    misc.get_route("clear_config"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                )
            )

            if response.status_code == 200:
                print("[Info] Remote configuration cleared successfully.")
                self.clear_local_config()

            else:
                print(f"[Warning] Failed to clear remote configuration; status {response.status_code}: {response.text.strip()}")

        except RequestException as e:
            print(f"[Error] Failed to clear remote configuration: {e}")

    @Slot()
    def load_parameters_from_file(self) -> None:
        """Load parameters from a file."""

        if self.show_load_warning:
            response, remember_choice = show_message_box(
                title="Load Parameters from File",
                message="Loading parameters from a file will overwrite the current configuration. Do you want to continue?",
                icon=constants.ICONS.warning,
                buttons=[QMessageBox.Yes, QMessageBox.No],
                remember_choice_option=True,
            )

            if remember_choice:
                self.show_load_warning = response != QMessageBox.Yes

            if response == QMessageBox.No:
                print("[Info] Load parameters from file operation cancelled by user.")
                return

        else:
            print("[Info] Loading parameters from file without warning as per user preference.")

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
            constants.SM.write("local_autopilot_param_hash", Path(file_path).stem)
            constants.SM.write("current_autopilot_parameters", self.config)

            self.add_parameters()
            self.update_status_label()
            print(f"[Info] Loaded parameters from {file_path}.")

        except Exception as e:
            print(f"[Error] Unable to load parameters from file: {e}")

    @Slot()
    def save_parameters_to_file(self) -> None:
        """Save parameters to a file."""

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            caption="Load Parameters from File",
            directory=constants.AUTOPILOT_PARAMS_DIR.as_posix(),
            filter="JSON Files (*.json);;All Files (*)",
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
            param_config = deepcopy(self.config[key])
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

    @Slot(bool)
    def refresh_config(self, instance_changed: bool) -> None:
        """
        Refresh the configuration from the telemetry server, typically after switching telemetry server instances.
        
        Parameters
        ----------
        instance_changed
            A boolean indicating whether the telemetry server instance has changed.
            If ``False``, the config will not be refreshed.
        """

        if not instance_changed:
            return

        try:
            remote_hash = constants.REQ_SESSION.get(
                urljoin(
                    misc.get_route("get_current_hash"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                )
            ).text.strip().replace('"', "")

            if not remote_hash:
                self.config = {}
            
            elif remote_hash in [config.stem for config in constants.AUTOPILOT_PARAMS_DIR.glob("*.json")]:
                with open(constants.AUTOPILOT_PARAMS_DIR / f"{remote_hash}.json", mode="r", encoding="utf-8") as file:
                    self.config = json.load(file)
            
            else:
                self.config = constants.REQ_SESSION.get(
                    urljoin(
                        misc.get_route("get_default_autopilot_parameters"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    )
                ).json()

        except RequestException as e:
            print(f"[Error] Failed to refresh config after telemetry server instance switch: {e}")
            self.config = {}
        
        constants.SM.write("current_autopilot_parameters", self.config)
        constants.SM.write("local_autopilot_param_hash", remote_hash)

        self.add_parameters()
        self.pull_all_parameters()
        self.update_status_label()
        
    @Slot(str)
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

        local_hash = constants.SM.read_str("local_autopilot_param_hash")
        message_part = f"Showing config: {local_hash}" if local_hash else "No config loaded"

        if not search_text:
            self.status_label.setText(f"Showing all {visible_count} parameters | {message_part}")

        elif visible_count == 0:
            self.status_label.setText(f"No parameters match '{search_text}' | {message_part}")

        else:
            self.status_label.setText(f"Showing {visible_count} parameters matching '{search_text}' | {message_part}")


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
            tooltip="Send the current value of the parameter to the telemetry server.",
        )
        self.pull_button = misc.pushbutton_maker(
            "Pull",
            constants.ICONS.download,
            self.pull_value,
            max_width=100,
            min_height=30,
            is_clickable=True,
            tooltip="Overwrite local value with value from telemetry server.",
        )
        self.reset_button = misc.pushbutton_maker(
            "Reset",
            constants.ICONS.refresh,
            self.reset_value,
            max_width=100,
            min_height=30,
            is_clickable=False,
            tooltip="Reset to default value."
        )

        self.right_layout.addWidget(self.send_button)
        self.right_layout.addWidget(self.pull_button)
        self.right_layout.addWidget(self.reset_button)
        # endregion right layout

        self.main_layout.addLayout(self.left_layout, 70)  # 70% width
        self.main_layout.addLayout(self.right_layout, 30)  # 30% width

        self.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.setLineWidth(1)

    @Slot()
    def send_value(self) -> None:
        """
        Send the current value of the parameter to the telemetry endpoint.
        
        DONE
        """

        print(f"[Info] Sending value for {self.name}: {self.current_value}")
        try:
            existing_data = constants.REQ_SESSION.get(
                urljoin(
                    misc.get_route("get_autopilot_parameters"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                )
            ).json()

        except RequestException as e:
            print(f"[Error] Failed to fetch existing autopilot parameters. Cannot update {self.name}: {e}")
            return

        if isinstance(existing_data, dict):
            if self.name not in existing_data:
                response = show_message_box(
                    title="Parameter Not Found",
                    message=(
                        "The remote autopilot parameters have changed and no longer include "
                        f"{self.name}. How do you want to proceed?"
                    ),
                    icon=constants.ICONS.warning,
                    buttons=[
                        MessageBoxButton(
                            "do_nothing",
                            "Do nothing",
                            QMessageBox.ButtonRole.NoRole,
                        ),
                        MessageBoxButton(
                            "create_config",
                            "Create new config but don't switch to it",
                            QMessageBox.ButtonRole.ActionRole,
                        ),
                        MessageBoxButton(
                            "create_config_urgent",
                            "Create new config and switch to it",
                            QMessageBox.ButtonRole.YesRole,
                        ),
                    ],
                )
                
                if response == "create_config":
                    local_autopilot_params = constants.SM.read_dict("current_autopilot_parameters")
                    try:
                        response = constants.REQ_SESSION.post(
                            urljoin(
                                misc.get_route("create_config"),
                                str(constants.SM.read_int("telemetry_server_instance_id")),
                            ),
                            json=json.dumps(local_autopilot_params, indent=None)
                        )
                        status_message = response.text.strip().replace('"', "")

                        if response.status_code == 200:
                            print("[Info] New config created and switched to successfully.")
                        
                        else:
                            raise RequestException(status_message)
                        
                    except RequestException as e:
                        print(f"[Error] Failed to create new config on telemetry server: {e}")
                        return
                    
                elif response == "create_config_urgent":
                    local_autopilot_params = constants.SM.read_dict("current_autopilot_parameters")
                    try:
                        response = constants.REQ_SESSION.post(
                            urljoin(
                                misc.get_route("set_default_autopilot_parameters"),
                                str(constants.SM.read_int("telemetry_server_instance_id")),
                            ),
                            json=json.dumps(local_autopilot_params, indent=None),
                        )
                        status_message = response.text.strip().replace('"', "")

                        if response.status_code == 200:
                            print("[Info] New config created and switched to successfully.")
                        
                        else:
                            raise RequestException(status_message)
                        
                    except RequestException as e:
                        print(f"[Error] Failed to create new config on telemetry server: {e}")
                        return
                    
                else:
                    print(f"[Info] User chose to do nothing about missing parameter {self.name}.")
                    return

            elif isinstance(existing_data[self.name], dict) and "current" in existing_data[self.name]:
                existing_data[self.name]["current"] = self.current_value

            else:
                existing_data[self.name] = self.current_value

            try:
                constants.REQ_SESSION.post(
                    urljoin(
                        misc.get_route("set_autopilot_parameters"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    ),
                    json=json.dumps(existing_data, indent=None),
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

    @Slot()
    def pull_value(self) -> None:
        """Pull the current value of the parameter from the telemetry endpoint."""

        try:
            data = constants.REQ_SESSION.get(
                urljoin(
                    misc.get_route("get_autopilot_parameters"),
                    str(constants.SM.read_int("telemetry_server_instance_id")),
                )
            ).json()

            if self.name in data:
                self.current_value = data[self.name]
                
                if isinstance(self.modify_element, QLineEdit):
                    self.modify_element.setText(str(self.current_value))
                
                elif self.value_display:
                    self.value_display.setText(str(self.current_value))
                print(f"[Info] Pulled {self.name} with value {self.current_value}.")

                temp_params = constants.SM.read_dict("current_autopilot_parameters")
                temp_params[self.name] = {"current": self.current_value, "description": self.description}
                constants.SM.write("current_autopilot_parameters", temp_params)

            else:
                response = show_message_box(
                    title="Parameter Not Found",
                    message=(
                        "The remote autopilot parameters have changed and no longer include "
                        f"{self.name}. How do you want to proceed?"
                    ),
                    icon=constants.ICONS.warning,
                    buttons=[
                        MessageBoxButton(
                            "do_nothing",
                            "Do nothing",
                            QMessageBox.ButtonRole.NoRole,
                        ),
                        MessageBoxButton(
                            "use_remote",
                            "Purge local config and use remote config",
                            QMessageBox.ButtonRole.ActionRole,
                        ),
                    ],
                )

                if response == "use_remote":
                    print("[Info] Replacing local config with remote config.")
                    

        except RequestException as e:
            response = show_message_box(
                title="Failed to Pull Parameter",
                message=(
                    f"Failed to pull value for {self.name} from telemetry server: {e}. "
                    "This may be due to a network error or a mismatch between the local "
                    "configuration and the telemetry server's configuration. Do you want to "
                    "pull the updated configuration from the telemetry server?"
                ),
                icon=constants.ICONS.warning,
                buttons=[QMessageBox.Yes, QMessageBox.No],
            )

            if response == QMessageBox.Yes:
                print("[Info] Pulling updated configuration from telemetry server.")
                

        self.reset_button.setEnabled(True)
        self.send_button.setEnabled(False)
        self.pull_button.setEnabled(False)

    @Slot()
    def reset_value(self) -> None:
        """Reset the value of the parameter to its default value."""

        self.current_value = deepcopy(self.default_val)
        if isinstance(self.modify_element, QLineEdit):
            self.modify_element.setText(str(self.current_value))
            print(f"[Info] {self.name} reset to default value: {self.current_value}.")

        elif self.value_display:
            self.value_display.setText(str(self.current_value))
            print(f"[Info] {self.name} reset to default value: {self.current_value}.")

        temp_params = constants.SM.read_dict("current_autopilot_parameters")
        temp_params[self.name] = {"current": self.current_value, "description": self.description}
        constants.SM.write("current_autopilot_parameters", temp_params)

        self.reset_button.setEnabled(False)
        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)

    @Slot()
    def update_value_from_lineedit(self) -> None:
        """
        Update value from ``QLineEdit`` input.

        Raises
        ------
        TypeError
            If the edited data is not of the expected type.
        """

        try:
            text = self.modify_element.text()

            # region string handling
            if self.type is str:
                try:
                    edited_data = literal_eval(text)

                except (ValueError, SyntaxError):
                    edited_data = text

            else:
                try:
                    edited_data = literal_eval(text)

                except (ValueError, SyntaxError) as e:
                    raise TypeError(f"Invalid literal for type {self.type.__name__}: {text}") from e
            # endregion string handling

            if not isinstance(edited_data, self.type):
                if self.type is bool and (isinstance(edited_data, (int, str)) and edited_data in {0, 1, "true", "false"}):
                    edited_data = bool(edited_data)

                elif self.type is float and isinstance(edited_data, int):
                    edited_data = float(edited_data)

                elif self.type is int and isinstance(edited_data, float) and edited_data.is_integer():
                    edited_data = int(edited_data)

                elif not isinstance(edited_data, self.type):
                    raise TypeError(f"Edited data must be of type {self.type.__name__}, but got {type(edited_data).__name__}.")

            temp_params = constants.SM.read_dict("current_autopilot_parameters")
            temp_params[self.name] = {"current": edited_data, "default": self.default_val, "description": self.description}
            constants.SM.write("current_autopilot_parameters", temp_params)

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

    @Slot()
    def edit_grouped_data(self) -> None:
        """Open a text editor for editing a sequence of values."""

        try:
            initial_text = json.dumps(self.current_value, indent=2)
            self.text_edit_window = TextEditWindow(highlighter=syntax_highlighters.JsonHighlighter, initial_text=initial_text)
            self.text_edit_window.setWindowTitle(f"Edit {self.name}")
            self.text_edit_window.user_text_emitter.connect(self.edit_grouped_data_callback)
            self.text_edit_window.show()

        except Exception as e:
            print(f"[Error] Failed to open text edit window for {self.name}: {e}")

    @Slot(str)
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
            edited_data = literal_eval(text)

            if edited_data == self.current_value:
                return

            if not isinstance(edited_data, self.type):
                raise TypeError(f"Edited data must be of type {self.type.__name__}, but got {type(edited_data).__name__}.")
            
            temp_params = constants.SM.read_dict("current_autopilot_parameters")
            temp_params[self.name] = {"current": edited_data, "default": self.default_val, "description": self.description}
            constants.SM.write("current_autopilot_parameters", temp_params)

        except (ValueError, TypeError):
            print(f"[Error] Invalid value for {self.name}. Resetting to previous value.")
            self.value_display.setText(str(self.current_value))
            return

        self.current_value = edited_data
        self.value_display.setText(str(self.current_value))

        self.send_button.setEnabled(True)
        self.pull_button.setEnabled(True)
        self.reset_button.setEnabled(True)
