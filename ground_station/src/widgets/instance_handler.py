import constants
import thread_classes
import requests
from urllib.parse import urljoin
from datetime import datetime
from typing import Any
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QWidget,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QScrollArea,
    QLineEdit,
    QLabel,
    QPushButton,
    QFormLayout,
    QTabWidget,
    QMessageBox,
)


class InstanceHandler(QWidget):
    """
    A widget to handle instances of the application. <br>

    This widget is responsible for providing a interface that allows users to
    select a instance of the simulation to connect to. Additionally, this interface
    will allow users to manage instances that are no longer running but still available.

    Inherits
    --------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.ids: list[int] = []
        self.widgets_by_id: dict[int, InstanceWidget] = {}

        self.current_search_text: str = ""
        self.timer = constants.copy_qtimer(constants.ONE_SECOND_TIMER)

        self.main_layout = QGridLayout()
        self.setLayout(self.main_layout)

        self.instances_container = QWidget()
        self.instances_layout = QVBoxLayout()

        self.instances_layout.setAlignment(Qt.AlignTop)
        self.instances_container.setLayout(self.instances_layout)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.instances_container)

        self.searchbar = QLineEdit()
        self.searchbar.setClearButtonEnabled(True)
        self.searchbar.setPlaceholderText("Search instances...")
        self.searchbar.textChanged.connect(self.filter_instances)

        # status label to show search results
        self.status_label = QLabel()
        self.status_label.setStyleSheet("color: #D3D3D3; font-size: 12pt;")
        self.status_label.setAlignment(Qt.AlignCenter)

        self.main_layout.addWidget(self.searchbar, 0, 0)
        self.main_layout.addWidget(self.status_label, 1, 0)
        self.main_layout.addWidget(self.scroll, 2, 0)

        self.instance_fetcher = thread_classes.InstanceFetcher()
        self.timer.timeout.connect(self.update_instances_starter)

        self.instance_fetcher.instances_fetched.connect(self.update_instances)
        self.instance_fetcher.request_url_change.connect(self.instance_fetch_failure)

        self.timer.start()

    def update_instances_starter(self) -> None:
        """Start the instance fetcher thread."""

        if not self.instance_fetcher.isRunning():
            self.instance_fetcher.start()

    def update_instances(self, instances: list[dict]) -> None:
        """
        Update the instance widgets based on the fetched instances.

        Parameters
        ----------
        instances
            A list of dictionaries containing instance data.
        """

        self.instances_container.setUpdatesEnabled(False)

        new_ids = {instance["instance_id"] for instance in instances if instance.get("instance_id") is not None}
        existing_ids = set(self.widgets_by_id.keys())

        # only remove widgets that are no longer present in the fetched instances
        for removed_id in existing_ids - new_ids:
            widget = self.widgets_by_id.pop(removed_id)
            self.instances_layout.removeWidget(widget)
            widget.deleteLater()

        for instance in instances:
            instance_id: int = instance.get("instance_id")
            updated_at: str = instance.get("updated_at")
            instance_identifier: str = instance.get("instance_identifier")

            if None in (instance_id, updated_at, instance_identifier):
                print(f"[Warning] Skipping instance with missing data: {instance}")
                continue

            widget = self.widgets_by_id.get(instance_id)

            if widget is not None:
                widget.updated_at_label.setText(updated_at)
                if widget.instance_identifier != instance_identifier:
                    widget.instance_name_edit.setText(instance_identifier)

            else:
                try:
                    new_widget = InstanceWidget(instance)
                    self.instances_layout.addWidget(new_widget)
                    self.widgets_by_id[instance_id] = new_widget

                except ValueError as e:
                    print(f"[Warning] Skipping invalid instance data: {e}")

        if not self.widgets_by_id:
            self.instances_container.setUpdatesEnabled(True)
            self.instances_container.update()
            self.handle_no_instances()

        elif len(self.widgets_by_id) == 1 and constants.TELEMETRY_SERVER_INSTANCE_ID == -1:
            only_id = next(iter(self.widgets_by_id))
            constants.TELEMETRY_SERVER_INSTANCE_ID = only_id
            self.widgets_by_id[only_id].setStyleSheet(self.widgets_by_id[only_id].activated_style_sheet)
            print(f"[Info] Only one instance found. Automatically connected to instance {only_id}.")
        
        elif len(self.widgets_by_id) > 1 and constants.TELEMETRY_SERVER_INSTANCE_ID == -1:
            self.instances_container.setUpdatesEnabled(True)
            self.instances_container.update()
            self.handle_multiple_instances()
            
        for widget in self.widgets_by_id.values():
            if widget.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
                widget.setStyleSheet(widget.activated_style_sheet)
            else:
                widget.setStyleSheet(widget.style_sheet)

        if self.current_search_text:
            self.filter_instances(self.current_search_text)
        else:
            self.status_label.setText(f"{len(self.widgets_by_id)} instances found")

        self.instances_container.setUpdatesEnabled(True)
        self.instances_container.update()

    def handle_no_instances(self) -> None:
        """Handle scenario when no valid instances exist."""

        self.timer.stop()
        create_new_instance = constants.show_message_box(
            "No Valid Instances",
            "No valid instances found. Would you like to create a new instance?",
            constants.ICONS.question,
            [
                QMessageBox.StandardButton.Yes,
                QMessageBox.StandardButton.No,
            ],
        )
        if create_new_instance == QMessageBox.StandardButton.Yes:
            try:
                new_instance_id: int = constants.REQ_SESSION.get(
                    constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()

                constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id
                instance_info: dict[str, Any] = constants.REQ_SESSION.get(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_instance_info"], str(new_instance_id)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()

                new_widget = InstanceWidget(instance_info)
                self.instances_layout.addWidget(new_widget)
                self.widgets_by_id[new_instance_id] = new_widget
                new_widget.setStyleSheet(new_widget.activated_style_sheet)
                print(f"[Info] Created new instance with ID {new_instance_id}.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to create a new instance: {e}")

        else:
            print("[Info] User chose not to create a new instance even though none were found.")

        self.timer.start()

    def handle_multiple_instances(self) -> None:
        """Handle scenario when multiple instances exist but none is selected."""

        self.timer.stop()

        # focus the instance handler tab so we can see the instances
        main_window = self.window()
        tab_widget = main_window.centralWidget()
        if isinstance(tab_widget, QTabWidget):
            tab_widget.setCurrentWidget(self)
        
        while constants.TELEMETRY_SERVER_INSTANCE_ID == -1:
            new_id = constants.show_input_dialog(
                "Select Instance",
                "Please select the instance you want to connect to.",
                input_type=int,
            )

            if new_id in self.widgets_by_id and new_id is not None:
                constants.TELEMETRY_SERVER_INSTANCE_ID = new_id
                self.widgets_by_id[new_id].setStyleSheet(self.widgets_by_id[new_id].activated_style_sheet)
                print(f"[Info] Connected to instance {new_id}")

        self.timer.start()

    def filter_instances(self, text: str) -> None:
        """
        Filter the displayed instances based on the search text.

        Parameters
        ----------
        text
            The text to filter instances by.
        """

        self.current_search_text = text

        for widget in self.widgets_by_id.values():
            if text.lower() in widget.instance_identifier.lower():
                widget.show()
            else:
                widget.hide()

        visible_count = sum(widget.isVisible() for widget in self.widgets_by_id.values())
        if text:
            self.status_label.setText(f"{visible_count} instances found matching '{text}'")
        else:
            self.status_label.setText(f"{len(self.widgets_by_id)} instances found")

    def instance_fetch_failure(self, telemetry_status: constants.TelemetryStatus) -> None:
        """
        Handles the case where the instance fetched fails.

        Parameters
        ----------
        telemetry_status
            <ul>
            <li> <code>SUCCESS</code> indicates that the telemetry server is reachable and instances were fetched successfully.</li>
            <li> <code>FAILURE</code> indicates that the telemetry server is not reachable and instances could not be fetched.</li>
            </ul>
        """

        if telemetry_status == constants.TelemetryStatus.FAILURE:
            print("[Warning] Failed to fetch instances from the telemetry server.")


class InstanceWidget(QFrame):
    """
    A widget to represent a single instance of the application.<br>

    This widget is responsible for displaying the information of a single instance
    and providing controls to connect or disconnect from it.

    Parameters
    ----------
    instance_info
        A dictionary containing information about the instance. It should include (at least):
        - `instance_id`: The unique identifier for the instance.
        - `instance_identifier`: A human-readable name for the instance.
        - `created_at`: The timestamp when the instance was created.
        - `updated_at`: The timestamp when the instance was last updated.

    Inherits
    --------
    `QFrame`
    """

    def __init__(self, instance_info: dict) -> None:
        super().__init__()

        self.instance_info = instance_info

        try:
            self.instance_id: int = instance_info["instance_id"]
            self.instance_identifier: str = instance_info["instance_identifier"]
            self.created_at: datetime = datetime.fromisoformat(instance_info["created_at"])
            self.updated_at: datetime = datetime.fromisoformat(instance_info["updated_at"])

            assert isinstance(self.instance_id, int), "Instance ID must be an integer."
            assert isinstance(self.instance_identifier, str), "Instance identifier must be a string."
            assert isinstance(self.created_at, datetime), "Created at must be a datetime object."
            assert isinstance(self.updated_at, datetime), "Updated at must be a datetime object."

        except (KeyError, AssertionError) as e:
            raise ValueError(f"Invalid instance information provided: {e}. Ensure the instance info contains the required fields.")

        self.main_layout = QHBoxLayout()
        self.main_layout.setContentsMargins(10, 10, 10, 10)

        # region setup widget style
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)

        self.style_sheet = """
            QFrame { background-color: #2E2E2E; border-radius: 0px; border: 1px solid #444; }
            QLabel { color: white; font-size: 12pt; }
            QLineEdit { background-color: #3E3E3E; color: white; border: 1px solid #555; border-radius: 3px; padding: 5px; }
            QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px 12px; border-radius: 5px; }
            QPushButton:hover { background-color: #45A049; }
            QPushButton#deleteButton { background-color: #F44336; }
            QPushButton#deleteButton:hover { background-color: #D32F2F; }
        """

        # used if this instance is currently selected
        self.activated_style_sheet = """
            QFrame { background-color: #2E2E2E; border-radius: 0px; border: 1px solid #999; }
            QLabel { color: white; font-size: 12pt; }
            QLineEdit { background-color: #3E3E3E; color: white; border: 1px solid #555; border-radius: 3px; padding: 5px; }
            QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px 12px; border-radius: 5px; }
            QPushButton:hover { background-color: #45A049; }
            QPushButton#deleteButton { background-color: #F44336; }
            QPushButton#deleteButton:hover { background-color: #D32F2F; }
        """

        self.setStyleSheet(self.style_sheet)

        self.form_layout = QFormLayout()
        self.instance_id_label = QLabel(str(self.instance_id))
        self.instance_name_edit = QLineEdit(self.instance_identifier)
        self.instance_name_edit.editingFinished.connect(self.on_instance_name_changed)
        self.created_at_label = QLabel(self.created_at.strftime("%Y-%m-%d %H:%M:%S"))
        self.updated_at_label = QLabel(self.updated_at.strftime("%Y-%m-%d %H:%M:%S"))

        self.form_layout.addRow("Instance ID", self.instance_id_label)
        self.form_layout.addRow("Instance Name", self.instance_name_edit)
        self.form_layout.addRow("Created At", self.created_at_label)
        self.form_layout.addRow("Updated At", self.updated_at_label)
        # endregion setup widget style

        # region buttons
        self.button_layout = QVBoxLayout()

        self.connect_button = QPushButton("Connect")
        self.connect_button_stylesheet = """
                                        QPushButton {
                                            background-color: #4CAF50;  /* green */
                                            color: white;
                                            font-weight: bold;
                                            padding: 6px 12px;
                                            border-radius: 5px;
                                        }
                                        QPushButton:hover {
                                            background-color: #45A049;
                                        }
                                        """
        self.connect_button.setStyleSheet(self.connect_button_stylesheet)
        self.connect_button.clicked.connect(self.on_connect_clicked)

        self.delete_button = QPushButton("Delete")
        self.delete_button.setObjectName("deleteButton")
        self.delete_button_stylesheet = """
                                        QPushButton#deleteButton {
                                            background-color: #F44336;  /* red */
                                            color: white;
                                            font-weight: bold;
                                            padding: 6px 12px;
                                            border-radius: 5px;
                                        }
                                        QPushButton#deleteButton:hover {
                                            background-color: #D32F2F;
                                        }
                                        """
        self.delete_button.setStyleSheet(self.delete_button_stylesheet)
        self.delete_button.clicked.connect(self.on_delete_clicked)

        self.button_layout.addWidget(self.connect_button)
        self.button_layout.addWidget(self.delete_button)
        # endregion buttons

        self.main_layout.addLayout(self.form_layout)
        self.main_layout.addLayout(self.button_layout)
        self.setLayout(self.main_layout)

    def on_instance_name_changed(self) -> None:
        """Handle the instance name change event."""

        new_name = self.instance_name_edit.text().strip()

        if new_name and new_name != self.instance_identifier:
            try:
                constants.REQ_SESSION.post(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["set_instance_name"], f"{self.instance_id}/{new_name}"),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                self.instance_identifier = new_name
                print(f"[Info] Instance name updated to {self.instance_identifier}.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to update instance name: {e}")

    def on_connect_clicked(self) -> None:
        """Handle the connect button click event."""

        if self.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
            print("[Info] Already connected to this instance.")

        else:
            try:
                constants.REQ_SESSION.get(urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_instance_info"], str(self.instance_id)))

                constants.TELEMETRY_SERVER_INSTANCE_ID = self.instance_id
                print(f"[Info] Connected to instance {self.instance_identifier} ({self.instance_id}).")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to connect to instance {self.instance_identifier}: {e}")

    def on_delete_clicked(self) -> None:
        """Handle the delete button click event."""

        if self.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
            try:
                new_instance_id = constants.REQ_SESSION.get(
                    constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()
                constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id

                constants.REQ_SESSION.delete(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["delete_instance"], str(self.instance_id)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                print(
                    f"[Info] Instance {self.instance_id} deleted and new instance created with ID {constants.TELEMETRY_SERVER_INSTANCE_ID}."
                )

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to delete instance {self.instance_id}: {e}")

        else:
            try:
                constants.REQ_SESSION.delete(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["delete_instance"], str(self.instance_id)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                print(f"[Info] Instance {self.instance_id} deleted successfully.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to delete instance {self.instance_id}: {e}")
