import json
from collections.abc import Callable
from datetime import datetime, timezone
from enum import auto
from typing import Any
from urllib.parse import urljoin

from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)
from requests import RequestException
from strenum import StrEnum
from syntax_highlighters import JsonHighlighter
from utils import constants, misc, thread_classes
from widgets.popup_edit import TextEditWindow


class ConfigInfo:
    """
    A class to store information about a autopilot parameter configuration.

    Parameters
    ----------
    hash_value
        The hash value of the parameter configuration.
    description
        A description of the parameter configuration.
    created_at
        The creation timestamp of the parameter configuration.

    Raises
    ------
    ValueError
        If the provided hash information is invalid.
    """

    __slots__ = ("_hash_value", "_description", "_created_at")

    def __init__(self, data: dict[str, Any]) -> None:
        try:
            self._hash_value = str(data["config_hash"])
            self._description = str(data["description"])
            self._created_at = self._utc_to_local(datetime.fromisoformat(data["created_at"]))

        except Exception as e:
            raise ValueError("Invalid hash info!") from e
    
    def __str__(self) -> str:
        """Return a string representation of the ``ConfigInfo`` object."""

        return f"ConfigInfo(hash_value={self._hash_value}, description={self._description}, created_at={self._created_at})"
    
    @property
    def hash_value(self) -> str:
        """Get the hash value of the parameter configuration."""
        
        return self._hash_value
    
    @property
    def description(self) -> str:
        """Get the description of the parameter configuration."""
        
        return self._description
    
    @property
    def created_at(self) -> datetime:
        """Get the creation timestamp of the parameter configuration."""
        
        return self._created_at
    
    @staticmethod
    def _utc_to_local(utc_dt: datetime) -> datetime:
        """
        Convert a UTC datetime to local timezone.

        Parameters
        ----------
        utc_dt
            The UTC datetime to convert.

        Returns
        -------
        datetime
            The converted local datetime.
        """

        if utc_dt.tzinfo is None:
            utc_dt = utc_dt.replace(tzinfo=timezone.utc)

        return utc_dt.astimezone()
    
class AutopilotConfigManager(QWidget):
    """
    A widget to manage and display autopilot parameter configuration hashes.

    Inherits
    -------
    ``QWidget``
    """

    class SortBy(StrEnum):
        """
        Enum representing the options for sorting configuration hashes.

        Options
        -------
        HASH_VALUE
            Sort by hash value.
        DESCRIPTION
            Sort by description.
        """

        HASH_VALUE = auto()
        DESCRIPTION = auto()

    def __init__(self) -> None:
        super().__init__()

        self.timer = misc.copy_qtimer(constants.ONE_SECOND_TIMER)

        self.widgets_by_hash: dict[str, ConfigWidget] = {}
        self.sort_by = self.SortBy.DESCRIPTION

        self.current_search_text: str = ""

        self.main_layout = QGridLayout()
        self.setLayout(self.main_layout)

        self.configs_container = QWidget()
        self.configs_layout = QVBoxLayout()

        self.configs_layout.setAlignment(Qt.AlignTop)
        self.configs_container.setLayout(self.configs_layout)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.configs_container)

        self.searchbar = QLineEdit()
        self.searchbar.setClearButtonEnabled(True)
        self.searchbar.setPlaceholderText("Search configurations...")
        self.searchbar.textChanged.connect(self.filter_instances)

        sort_layout = QHBoxLayout()
        sort_layout.setContentsMargins(0, 0, 0, 0)
        sort_layout.setSpacing(5)

        self.sort_label = QLabel("Sort by:")
        self.sort_by_dropdown = QComboBox()
        self.sort_label.setBuddy(self.sort_by_dropdown)
        self.sort_by_dropdown.addItems([option.name for option in AutopilotConfigManager.SortBy])
        self.sort_by_dropdown.setCurrentText(self.sort_by.name)
        self.sort_by_dropdown.currentTextChanged.connect(self.on_sort_by_changed)

        sort_layout.addWidget(self.sort_label)
        sort_layout.addWidget(self.sort_by_dropdown)
        sort_layout.addStretch()

        # status label to show search results and connection status
        self.status_label = QLabel()
        self.status_label.setStyleSheet("color: #D3D3D3; font-size: 12pt;")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.update_status_label()

        self.button_groupbox = QGroupBox()
        self.button_layout = QHBoxLayout()

        self.manual_refresh_button = misc.pushbutton_maker(
            "Refresh Configs",
            constants.ICONS.refresh,
            self.hash_fetcher_starter
        )
        self.auto_refresh_toggle = QCheckBox("Auto Refresh?")
        self.auto_refresh_toggle.setChecked(True)
        self.auto_refresh_toggle.stateChanged.connect(self.on_auto_refresh_toggled)

        self.create_new_config_button = misc.pushbutton_maker(
            "Create New Config",
            constants.ICONS.add,
            self.create_new_config,
        )
        self.button_layout.addWidget(self.manual_refresh_button)
        self.button_layout.addWidget(self.create_new_config_button)
        self.button_groupbox.setLayout(self.button_layout)

        self.main_layout.addLayout(sort_layout, 0, 0)
        self.main_layout.addWidget(self.searchbar, 2, 0)
        self.main_layout.addWidget(self.status_label, 3, 0)
        self.main_layout.addWidget(self.scroll, 4, 0)
        self.main_layout.addWidget(self.button_groupbox, 5, 0)

        # put auto refresh toggle on the buttom center of the main layout
        self.main_layout.addWidget(self.auto_refresh_toggle, 6, 0, alignment=Qt.AlignCenter)

        self.hashes_fetcher = thread_classes.AutopilotThreadRouter.AvailableHashesFetcherThread()
        self.active_hash_fetcher = thread_classes.AutopilotThreadRouter.ActiveHashFetcherThread()

        self.hashes_fetcher.response.connect(self.on_hashes_fetched)
        self.active_hash_fetcher.response.connect(self.on_active_hash_fetched)

        self.timer.timeout.connect(self.hash_fetcher_starter)
        self.timer.timeout.connect(self.active_hash_fetcher_starter)

        self.on_sort_by_changed(self.sort_by.name)
        self.timer.start()

    def on_auto_refresh_toggled(self, state: int) -> None:
        """
        Handle the auto refresh toggle state change.

        Parameters
        ----------
        state
            The new state of the auto refresh toggle.
        """

        if state == Qt.Checked:
            self.timer.start()
        else:
            self.timer.stop()

    def on_hashes_fetched(self, request_result: tuple[list[dict[str, Any]], constants.TelemetryStatus]) -> None:
        """
        Handle the fetched hashes from the telemetry server.

        Parameters
        ----------
        request_result
            A tuple containing:
            - A list of dictionaries with information about each available configuration hash.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        available_hashes, status = request_result

        if status == constants.TelemetryStatus.SUCCESS:
            self.configs_container.setUpdatesEnabled(False)

            existing_hashes = set(self.widgets_by_hash.keys())
            new_hashes: set[str] = {hash_info["config_hash"] for hash_info in available_hashes}

            deprecated_hashes = existing_hashes - new_hashes

            for hash_value in deprecated_hashes:
                widget = self.widgets_by_hash.pop(hash_value)
                self.configs_layout.removeWidget(widget)
                widget.deleteLater()

            for hash_config in available_hashes:
                try:
                    hash_info = ConfigInfo(hash_config)

                except ValueError as e:
                    print(f"[Warning] Invalid hash info received from server: {e}")
                    continue
                
                widget = self.widgets_by_hash.get(hash_info.hash_value)
                if widget:
                    if widget.hash_description != hash_info.description:
                        widget.hash_description = hash_info.description
                        widget.hash_description_edit.setText(hash_info.description)

                else:
                    try:
                        widget = ConfigWidget(hash_info)
                        self.widgets_by_hash[hash_info.hash_value] = widget
                    
                    except Exception as e:
                        print(f"[Warning] Failed to create widget for hash {hash_info.hash_value}: {e}")

            for widget in sorted(self.widgets_by_hash.values(), key=self.sort_key):
                self.configs_layout.addWidget(widget)

                if widget.hash_value == constants.REMOTE_AUTOPILOT_PARAM_HASH:
                    widget.setStyleSheet(ConfigWidget.activated_style_sheet)
                else:
                    widget.setStyleSheet(ConfigWidget.style_sheet)

            if self.current_search_text:
                self.filter_instances(self.current_search_text)
            else:
                self.update_status_label()

            self.configs_container.setUpdatesEnabled(True)
            self.configs_container.update()

    def on_active_hash_fetched(self, request_result: tuple[str, constants.TelemetryStatus]) -> None:
        """
        Handle the fetched active configuration hash from the telemetry server.

        Parameters
        ----------
        request_result
            A tuple containing:
            - The active configuration hash as a string.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        hash_string, status = request_result

        if status == constants.TelemetryStatus.SUCCESS:
            constants.REMOTE_AUTOPILOT_PARAM_HASH = hash_string.strip().replace('"', '')

    def create_new_config(self) -> None:
        """
        Handle the create new configuration button click event.

        Opens a popup window to enter new configuration data.
        """

        self.text_edit_window = TextEditWindow(highlighter=JsonHighlighter)
        self.text_edit_window.setWindowTitle("Create New Autopilot Configuration")
        self.text_edit_window.user_text_emitter.connect(self.create_new_config_callback)
        self.text_edit_window.show()

    def create_new_config_callback(self, config_data: str) -> None:
        """
        Callback function to handle the new configuration data entered by the user.

        Parameters
        ----------
        config_data
            The JSON string representing the new configuration data.
        """

        self.timer.stop()

        try:
            if not config_data.strip():
                print("[Warning] No configuration data provided.")
                return
            
            config_dict = json.loads(config_data)

            response = constants.REQ_SESSION.post(
                constants.TELEMETRY_SERVER_ENDPOINTS["create_config"],
                json=config_dict,
            )
            
            if response.status_code != 200:
                raise RequestException(f"Server returned status code {response.status_code} with message: {response.text}")
        
        except json.JSONDecodeError as e:
            print(f"[Error] Invalid JSON data: {e}")
        
        except RequestException as e:
            print(f"[Error] Failed to create new configuration: {e}")

        self.timer.start()
    
    def on_sort_by_changed(self, sort_method: str) -> None:
        """
        Handle the sort by dropdown change event.

        Parameters
        ----------
        sort_method
            The new sort by option selected.
        """

        try:
            self.sort_by = AutopilotConfigManager.SortBy[sort_method]

            if self.sort_by == AutopilotConfigManager.SortBy.HASH_VALUE:
                self.sort_key: Callable[[ConfigWidget], str] = lambda widget: widget.hash_value

            elif self.sort_by == AutopilotConfigManager.SortBy.DESCRIPTION:
                self.sort_key: Callable[[ConfigWidget], str] = lambda widget: widget.hash_description

            if not self.timer.isActive():
                fake_request_result = (list(self.widgets_by_hash.keys()), constants.TelemetryStatus.SUCCESS)
                self.on_hashes_fetched(fake_request_result)

        except ValueError:
            print(f"[Warning] Invalid sort by option: {sort_method}")
            return
        
    def filter_instances(self, search_text: str) -> None:
        """
        Filter the displayed configuration widgets based on the search text.

        Parameters
        ----------
        search_text
            The text to filter the configurations by.
        """

        self.current_search_text = search_text.lower()
        visible_count = 0

        for widget in self.widgets_by_hash.values():
            if (
                self.current_search_text in widget.hash_value.lower()
                or self.current_search_text in widget.hash_description.lower()
            ):
                widget.show()
                visible_count += 1
            else:
                widget.hide()

        self.update_status_label(visible_count)

    def update_status_label(self, visible_count: int | None = None) -> None:
        """
        Update the status label with the current number of visible configurations.

        Parameters
        ----------
        visible_count
            The number of currently visible configuration widgets. If ``None``, counts all widgets.
        """

        if visible_count is None:
            visible_count = len(self.widgets_by_hash)

        total_count = len(self.widgets_by_hash)
        status_text = (
            f"Showing {visible_count} of {total_count} configurations | "
            f"Remote Hash: {constants.REMOTE_AUTOPILOT_PARAM_HASH}"
        )
        self.status_label.setText(status_text)

    def hash_fetcher_starter(self) -> None:
        """Refresh the list of available configuration hashes from the telemetry server."""

        if not self.hashes_fetcher.isRunning():
            self.hashes_fetcher.start()

    def active_hash_fetcher_starter(self) -> None:
        """Refresh the active configuration hash from the telemetry server."""

        if not self.active_hash_fetcher.isRunning():
            self.active_hash_fetcher.start()


class ConfigWidget(QFrame):
    """
    A widget to manage and display autopilot parameter configurations.

    Inherits
    -------
    ``QFrame``
    """

    style_sheet = """
            QFrame {
                background-color: #2E2E2E;
                border: 1px solid #444444;
                border-radius: 0px;
            }
            QLabel {
                color: white;
                font-size: 12pt;
            }
    """

    activated_style_sheet = """
            QFrame {
                background-color: #3E3E3E;
                border: 2px solid #D3D3D3;
                border-radius: 0px;
            }
            QLabel {
                color: #FFFFFF;
                font-size: 12pt;
            }
    """

    download_button_style_sheet = """
            QPushButton {
                background-color: #4CAF50;
                border: none;
                color: white;
                padding: 5px 10px;
                text-align: center;
                font-size: 12pt;
                margin: 2px 1px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
    """

    delete_button_style_sheet = """
            QPushButton {
                background-color: #f44336;
                border: none;
                color: white;
                padding: 5px 10px;
                text-align: center;
                font-size: 12pt;
                margin: 2px 1px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
    """

    def __init__(self, config_info: ConfigInfo) -> None:
        super().__init__()

        self.hash_value = config_info.hash_value
        self.hash_description = config_info.description
        self.hash_created_at = config_info.created_at

        self.main_layout = QHBoxLayout()
        self.main_layout.setContentsMargins(10, 10, 10, 10)

        # region setup widget style
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setStyleSheet(ConfigWidget.style_sheet)

        self.form_layout = QFormLayout()
        self.hash_value_label = QLabel(self.hash_value)
        self.hash_created_at_label = QLabel(self.hash_created_at.strftime("%Y-%m-%d %I:%M:%S %p"))

        self.hash_description_edit = QLineEdit(self.hash_description)
        self.hash_description_edit.editingFinished.connect(self.on_description_changed)


        self.form_layout.addRow("Description:", self.hash_description_edit)
        self.form_layout.addRow("Hash Value:", self.hash_value_label)
        self.form_layout.addRow("Created At:", self.hash_created_at_label)
        # endregion setup widget style

        # region buttons
        self.button_layout = QVBoxLayout()

        self.download_button = misc.pushbutton_maker(
            "Download Config",
            constants.ICONS.download,
            self.on_download_clicked,
            style_sheet=ConfigWidget.download_button_style_sheet,
        )

        self.delete_button = misc.pushbutton_maker(
            "Delete Config",
            constants.ICONS.delete,
            self.on_delete_clicked,
            style_sheet=ConfigWidget.delete_button_style_sheet,
        )

        self.button_layout.addWidget(self.download_button)
        self.button_layout.addWidget(self.delete_button)
        # endregion buttons

        self.main_layout.addLayout(self.form_layout)
        self.main_layout.addLayout(self.button_layout)
        self.setLayout(self.main_layout)

    def on_download_clicked(self) -> None:
        """Handle the download button click event."""

        print(f"[Info] Downloading configuration with hash: {self.hash_value}")

        for hash_value in constants.AUTOPILOT_PARAMS_DIR.iterdir():
            if hash_value.name == self.hash_value:
                print(f"[Info] Configuration {self.hash_value} already exists locally.")
                return
            
        try:
            data = constants.REQ_SESSION.get(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS['get_config_from_hash'],
                        self.hash_value
                )
            ).json()

            if not isinstance(data, dict):
                raise TypeError
            
            file_name = f"{self.hash_value}.json"
            config_path = constants.AUTOPILOT_PARAMS_DIR / file_name
            with open(config_path, "w") as config_file:
                json.dump(data, config_file, indent=4)
            
            print("[Info] Configuration downloaded successfully!")
            
        except RequestException as e:
            print(f"[Error] Failed to download configuration: {e}")
        
        except TypeError as e:
            print(f"[Error] Invalid data format received from server, expected `dict` but got `{data}`: {e}")

    def on_delete_clicked(self) -> None:
        """Handle the delete button click event."""

        print(f"[Info] Deleting configuration with hash: {self.hash_value}")

        try:
            response = constants.REQ_SESSION.delete(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS['delete_config'],
                    self.hash_value
                )
            )

            if response.status_code == 200:
                print(f"[Info] Configuration {self.hash_value} deleted successfully!")

            else:
                raise RequestException(f"Server returned status code {response.status_code} with message: {response.text}")

        except RequestException as e:
            print(f"[Error] Failed to delete configuration: {e}")

    def on_description_changed(self) -> None:
        """Handle the description edit change event."""

        new_description = self.hash_description_edit.text().strip()

        if new_description not in {"", self.hash_description}:
            try:
                url = urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["set_hash_description"],
                    f"{self.hash_value}/{new_description}"
                )
                response = constants.REQ_SESSION.post(url)

                if response.status_code == 200:
                    self.hash_description = new_description
                    print(f"[Info] Description for config {self.hash_value} updated successfully!")

                else:
                    raise RequestException(f"Server returned status code {response.status_code} with message: {response.text}")

            except RequestException as e:
                print(f"[Error] Failed to update description: {e}")
