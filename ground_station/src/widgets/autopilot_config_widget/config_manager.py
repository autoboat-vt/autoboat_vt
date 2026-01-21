import json
from collections.abc import Callable
from enum import auto
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
from utils import constants, misc
from utils.thread_classes import AutopilotThreadRouter


class ConfigInfo:
    """
    A class to store information about a autopilot parameter configuration.

    Parameters
    ----------
    hash_value
        The hash value of the parameter configuration.
    description
        A description of the parameter configuration.

    Raises
    ------
    ValueError
        If the provided hash information is invalid.
    """

    __slots__ = ("_hash_value", "_description")

    def __init__(self, hash_value: str, description: str) -> None:
        try:
            self._hash_value = str(hash_value)
            self._description = str(description)

        except Exception as e:
            raise ValueError("Invalid hash info!") from e
    
    @property
    def hash_value(self) -> str:
        """Get the hash value of the parameter configuration."""
        
        return self._hash_value
    
    @property
    def description(self) -> str:
        """Get the description of the parameter configuration."""
        
        return self._description
    
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
        self.auto_refresh_toggle.setChecked(False)
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

        self.on_sort_by_changed(self.sort_by.name)

        self.hash_checker = AutopilotThreadRouter.AvailableHashesFetcherThread()
        self.hash_checker.response.connect(self.on_hashes_fetched)
        self.timer.timeout.connect(self.hash_fetcher_starter)

    def on_hashes_fetched(self, request_result: tuple[list[str], constants.TelemetryStatus]) -> None:
        """
        Handle the fetched hashes from the telemetry server.

        Parameters
        ----------
        request_result
            A tuple containing:
            - A list of available configuration hashes.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        available_hashes, status = request_result

        if status == constants.TelemetryStatus.SUCCESS:
            self.configs_container.setUpdatesEnabled(False)

            for hash_value in list(self.widgets_by_hash.keys()):
                widget = self.widgets_by_hash.pop(hash_value)
                self.configs_layout.removeWidget(widget)
                widget.deleteLater()

            for hash_value in available_hashes:
                if hash_value not in self.widgets_by_hash:
                    try:
                        config_info = ConfigInfo(hash_value, "")

                    except ValueError as e:
                        print(f"[Warning] Skipping invalid config info: {e}")
                        continue

                    new_widget = ConfigWidget(config_info)
                    self.configs_layout.addWidget(new_widget)
                    self.widgets_by_hash[hash_value] = new_widget

            for widget in sorted(self.widgets_by_hash.values(), key=self.sort_key):
                self.configs_layout.addWidget(widget)

                if widget.hash_value == constants.AUTOPILOT_PARAM_HASH:
                    widget.setStyleSheet(ConfigWidget.activated_style_sheet)
                else:
                    widget.setStyleSheet(ConfigWidget.style_sheet)

            if self.current_search_text:
                self.filter_instances(self.current_search_text)
            else:
                self.update_status_label()

            self.configs_container.setUpdatesEnabled(True)
            self.configs_container.update()

    def create_new_config(self) -> None:
        """
        Handle the create new configuration button click event.

        TODO: Implement creating a new configuration
        """
        
        pass
    
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
            The number of currently visible configuration widgets. If None, counts all widgets.
        """

        if visible_count is None:
            visible_count = len(self.widgets_by_hash)

        total_count = len(self.widgets_by_hash)
        self.status_label.setText(f"Showing {visible_count} of {total_count} configurations.")

    def hash_fetcher_starter(self) -> None:
        """Refresh the list of available configuration hashes from the telemetry server."""

        if not self.hash_checker.isRunning():
            self.hash_checker.start()

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
                border-radius: 5px;
            }
            QLabel {
                color: #FFFFFF;
                font-size: 12pt;
            }
    """

    activated_style_sheet = """
            QFrame {
                background-color: #3E3E3E;
                border: 2px solid #1E90FF;
                border-radius: 5px;
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
                font-size: 10pt;
                margin: 2px 1px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
    """

    update_description_button_style_sheet = """
            QPushButton {
                background-color: #008CBA;
                border: none;
                color: white;
                padding: 5px 10px;
                text-align: center;
                font-size: 10pt;
                margin: 2px 1px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #007bb5;
            }
    """

    download_description_button_style_sheet = """
            QPushButton {
                background-color: #f44336;
                border: none;
                color: white;
                padding: 5px 10px;
                text-align: center;
                font-size: 10pt;
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

        self.main_layout = QHBoxLayout()
        self.main_layout.setContentsMargins(10, 10, 10, 10)

        # region setup widget style
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setStyleSheet(ConfigWidget.style_sheet)

        self.form_layout = QFormLayout()
        self.hash_value_label = QLabel(self.hash_value)

        self.hash_description_edit = QLineEdit(self.hash_description)

        self.form_layout.addRow("Hash Value:", self.hash_value_label)
        self.form_layout.addRow("Description:", self.hash_description_edit)
        # endregion setup widget style

        # region buttons
        self.button_layout = QGridLayout()

        self.download_button = misc.pushbutton_maker(
            "Download Config",
            constants.ICONS.download,
            self.on_download_clicked,
            style_sheet=ConfigWidget.download_button_style_sheet,
        )

        self.update_description_button = misc.pushbutton_maker(
            "Update Description",
            constants.ICONS.upload,
            self.on_update_description_clicked,
            style_sheet=ConfigWidget.update_description_button_style_sheet,
        )

        self.download_description_button = misc.pushbutton_maker(
            "Download Description",
            constants.ICONS.download,
            self.on_download_description_clicked,
            style_sheet=ConfigWidget.download_description_button_style_sheet,
        )

        self.button_layout.addWidget(self.download_button, 0, 0, 1, 2)
        self.button_layout.addWidget(self.update_description_button, 1, 0)
        self.button_layout.addWidget(self.download_description_button, 1, 1)
        # endregion buttons

        self.main_layout.addLayout(self.form_layout)
        self.main_layout.addLayout(self.button_layout)
        self.setLayout(self.main_layout)

    def on_download_clicked(self) -> None:
        """Handle the download button click event."""

        print(f"[Info] Downloading configuration with hash: {self.hash_value}")

        if self.hash_value == constants.AUTOPILOT_PARAM_HASH:
            print("[Info] This configuration is already active.")

        for hash_value in constants.AUTOPILOT_PARAMS_DIR.iterdir():
            if hash_value.name == self.hash_value:
                print(f"[Info] Configuration {self.hash_value} already exists locally.")
                return
            
        try:
            data = constants.REQ_SESSION.get(
                    urljoin(
                        constants.TELEMETRY_SERVER_URL,
                        f"{constants.TELEMETRY_SERVER_ENDPOINTS['get_config_from_hash'] + self.hash_value}"
                )
            ).json()

            if not isinstance(data, dict):
                raise TypeError
            
            file_name = f"{self.hash_value}.json"
            config_path = constants.AUTOPILOT_PARAMS_DIR / file_name
            with open(config_path, "w") as config_file:
                json.dump(data, config_file, indent=4)
            
            print("[Info] Configuration downloaded successfully!.")
            
        except RequestException as e:
            print(f"[Error] Failed to download configuration: {e}")
        
        except TypeError as e:
            print(f"[Error] Invalid data format received from server, expected `dict` but got `{data}`: {e}")

    def on_update_description_clicked(self) -> None:
        """Handle the update description button click event."""

        new_description = self.hash_description_edit.text()
        try:
            constants.REQ_SESSION.post(
                urljoin(
                    constants.TELEMETRY_SERVER_URL,
                    f"{constants.TELEMETRY_SERVER_ENDPOINTS['set_hash_description'] + self.hash_value}/{new_description}",
                ),
            )
            print(f"[Info] Updated description for hash {self.hash_value} to: {new_description}")
        
        except RequestException as e:
            print(f"[Error] Failed to update description: {e}")

    def on_download_description_clicked(self) -> None:
        """Handle the download description button click event."""

        try:
            response = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_URL,
                    f"{constants.TELEMETRY_SERVER_ENDPOINTS['get_hash_description'] + self.hash_value}",
                ),
            )

            if not response.status_code == 200:
                raise RequestException(f"Server returned status code {response.status_code}")
            
            if not isinstance(response.text, str):
                raise TypeError
            
            self.hash_description_edit.setText(response.text)
            print(f"[Info] Downloaded description for hash {self.hash_value}: {response.text}")

        except TypeError as e:
            print(f"[Warning] Invalid description format received from server: {e}")
        
        except RequestException as e:
            print(f"[Warning] Failed to download description: {e}")
