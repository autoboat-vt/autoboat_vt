import requests
from utils import constants, thread_classes, misc
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
    QFormLayout,
    QTabWidget,
    QMessageBox,
    QGroupBox,
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
        self.timer = misc.copy_qtimer(constants.ONE_SECOND_TIMER)

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

        # status label to show search results and connection status
        self.status_label = QLabel()
        self.status_label.setStyleSheet("color: #D3D3D3; font-size: 12pt;")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.update_status_label()

        self.button_groupbox = QGroupBox()
        self.button_layout = QHBoxLayout()

        self.delete_all_button = misc.pushbutton_maker(
            "Delete All Instances",
            constants.ICONS.delete,
            self.delete_all_instances,
            max_width=constants.WINDOW_BOX.width() // 2,
        )
        self.create_instance_button = misc.pushbutton_maker(
            "Create New Instance",
            constants.ICONS.add,
            self.create_new_instance,
            max_width=constants.WINDOW_BOX.width() // 2,
        )

        self.button_layout.addWidget(self.delete_all_button)
        self.button_layout.addWidget(self.create_instance_button)
        self.button_groupbox.setLayout(self.button_layout)

        self.main_layout.addWidget(self.searchbar, 0, 0)
        self.main_layout.addWidget(self.status_label, 1, 0)
        self.main_layout.addWidget(self.scroll, 2, 0)
        self.main_layout.addWidget(self.button_groupbox, 3, 0)

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
        constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = False

        # region update and create new instance widgets
        new_ids = {instance["instance_id"] for instance in instances if isinstance(instance.get("instance_id"), int)}
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

            # checking if we have seen this instance before
            widget = self.widgets_by_id.get(instance_id)
            if widget:
                new_updated_at = datetime.fromisoformat(updated_at).strftime("%Y-%m-%d %I:%M:%S %p")
                widget.updated_at_label.setText(new_updated_at)
                if widget.instance_identifier != instance_identifier:
                    widget.instance_name_edit.setText(instance_identifier)

            else:
                try:
                    new_widget = InstanceWidget(instance)
                    self.instances_layout.addWidget(new_widget)
                    self.widgets_by_id[instance_id] = new_widget

                except ValueError as e:
                    print(f"[Warning] Skipping invalid instance data: {e}")

        # endregion update and create new instance widgets

        # region instance selection logic
        if len(self.widgets_by_id) == 0:
            self.instances_container.setUpdatesEnabled(True)
            self.instances_container.update()
            self.handle_no_instances()

        elif constants.TELEMETRY_SERVER_INSTANCE_ID == -1:
            self.instances_container.setUpdatesEnabled(True)
            self.instances_container.update()

        # endregion instance selection logic

        for widget in self.widgets_by_id.values():
            if widget.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
                widget.setStyleSheet(InstanceWidget.activated_style_sheet)
            else:
                widget.setStyleSheet(InstanceWidget.style_sheet)

        if self.current_search_text:
            self.filter_instances(self.current_search_text)
        else:
            self.update_status_label()

        self.instances_container.setUpdatesEnabled(True)
        self.instances_container.update()

    def handle_no_instances(self) -> None:
        """Handle scenario when no valid instances exist."""

        self.timer.stop()
        create_new_instance = misc.show_message_box(
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

                instance_info: dict[str, Any] = constants.REQ_SESSION.get(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_instance_info"], str(new_instance_id)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()

                new_widget = InstanceWidget(instance_info)
                self.instances_layout.addWidget(new_widget)
                self.widgets_by_id[new_instance_id] = new_widget

                constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True
                constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id
                print(f"[Info] Created new instance with ID #{new_instance_id}.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to create a new instance: {e}")

            except ValueError as e:
                print(f"[Error] Failed to create instance widget: {e}")

        else:
            print("[Info] User chose not to create a new instance even though none were found.")

        self.timer.start()

    def update_status_label(self) -> None:
        """Update the status label with current connection status and instance count."""

        instance_count = len(self.widgets_by_id)

        if constants.TELEMETRY_SERVER_INSTANCE_ID == -1:
            status_prefix = "NOT CONNECTED - Please select an instance | "
        else:
            connected_widget = self.widgets_by_id.get(constants.TELEMETRY_SERVER_INSTANCE_ID)
            if connected_widget:
                status_prefix = f"✓ Connected to: {connected_widget.instance_identifier} (ID: {constants.TELEMETRY_SERVER_INSTANCE_ID}) | "
            else:
                status_prefix = f"✓ Connected to instance #{constants.TELEMETRY_SERVER_INSTANCE_ID} | "

        if self.current_search_text:
            visible_count = sum(widget.isVisible() for widget in self.widgets_by_id.values())
            self.status_label.setText(
                f"{status_prefix}{visible_count} instances found matching '{self.current_search_text}'"
            )
        else:
            self.status_label.setText(f"{status_prefix}{instance_count} instances found")

    def handle_multiple_instances(self) -> None:
        """
        Handles the scenario when multiple instances exist but none is selected.
        This scenario should only occur when the appiclation is started for the first time
        as no instance is selected by default.
        """

        self.timer.stop()
        dialog_text = "Please select the instance you want to connect to."

        while True:
            new_id = misc.show_input_dialog(
                "Select Instance",
                dialog_text,
                input_type=int,
            )

            if new_id is None:
                dialog_text = "No instance selected. Please enter a valid instance ID."

            else:
                if new_id in self.widgets_by_id:
                    constants.TELEMETRY_SERVER_INSTANCE_ID = new_id
                    constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True
                    print(f"[Info] Connected to instance #{constants.TELEMETRY_SERVER_INSTANCE_ID}.")
                    break

                dialog_text = f"Instance #{new_id} not found. Please enter a valid instance ID."

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

        self.update_status_label()

    def create_new_instance(self) -> None:
        """Create a new instance on the telemetry server."""

        try:
            new_instance_id: int = constants.REQ_SESSION.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            instance_info: dict[str, Any] = constants.REQ_SESSION.get(
                urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_instance_info"], str(new_instance_id)),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            new_widget = InstanceWidget(instance_info)
            self.instances_layout.addWidget(new_widget)
            self.widgets_by_id[new_instance_id] = new_widget
            print(f"[Info] Created new instance with ID #{new_instance_id}.")

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to create a new instance: {e}")

        except ValueError as e:
            print(f"[Error] Failed to create instance widget: {e}")

    def delete_all_instances(self) -> None:
        """Delete all instances from the telemetry server."""

        try:
            constants.REQ_SESSION.delete(
                constants.TELEMETRY_SERVER_ENDPOINTS["delete_all_instances"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            )

            new_instance_id: int = constants.REQ_SESSION.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            print(f"[Info] All instances deleted. New instance created with ID #{new_instance_id}.")
            constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id
            constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to delete all instances: {e}")

    def instance_fetch_failure(self, telemetry_status: constants.TelemetryStatus) -> None:
        """
        Handles the case where fetching instances from the telemetry server fails.

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

    Attributes
    ----------
    style_sheet: `str`
        The default style sheet for the widget.

    activated_style_sheet: `str`
        The style sheet for the widget when it is the active instance.

    Raises
    ------
    ValueError
        If the `instance_info` dictionary does not contain the required fields or if they are of incorrect types.

    Inherits
    --------
    `QFrame`
    """

    style_sheet = """
            QFrame { background-color: #2E2E2E; border-radius: 0px; border: 1px solid #444; }
            QLabel { color: white; font-size: 12pt; }
            QLineEdit { background-color: #3E3E3E; color: white; border: 1px solid #555; border-radius: 3px; padding: 5px; }
            QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px 12px; border-radius: 5px; }
            QPushButton:hover { background-color: #45A049; }
            QPushButton#deleteButton { background-color: #F44336; }
            QPushButton#deleteButton:hover { background-color: #D32F2F; }
            """

    activated_style_sheet = """
            QFrame { background-color: #2E2E2E; border-radius: 0px; border: 1px solid #999; }
            QLabel { color: white; font-size: 12pt; }
            QLineEdit { background-color: #3E3E3E; color: white; border: 1px solid #555; border-radius: 3px; padding: 5px; }
            QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px 12px; border-radius: 5px; }
            QPushButton:hover { background-color: #45A049; }
            QPushButton#deleteButton { background-color: #F44336; }
            QPushButton#deleteButton:hover { background-color: #D32F2F; }
        """

    connect_button_stylesheet = """
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

    delete_button_stylesheet = """
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

    def __init__(self, instance_info: dict[str, Any]) -> None:
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
            raise ValueError(
                f"Invalid instance information provided: {e}. Ensure the instance info contains the required fields."
            )

        self.main_layout = QHBoxLayout()
        self.main_layout.setContentsMargins(10, 10, 10, 10)

        # region setup widget style
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setStyleSheet(InstanceWidget.style_sheet)

        self.form_layout = QFormLayout()
        self.instance_id_label = QLabel(str(self.instance_id))
        self.instance_name_edit = QLineEdit(self.instance_identifier)
        self.instance_name_edit.editingFinished.connect(self.on_instance_name_changed)
        self.created_at_label = QLabel(self.created_at.strftime("%Y-%m-%d %I:%M:%S %p"))
        self.updated_at_label = QLabel(self.updated_at.strftime("%Y-%m-%d %I:%M:%S %p"))

        self.form_layout.addRow("Instance ID", self.instance_id_label)
        self.form_layout.addRow("Instance Name", self.instance_name_edit)
        self.form_layout.addRow("Created At", self.created_at_label)
        self.form_layout.addRow("Updated At", self.updated_at_label)
        # endregion setup widget style

        # region buttons
        self.button_layout = QVBoxLayout()

        self.connect_button = misc.pushbutton_maker(
            "Connect",
            constants.ICONS.connect,
            self.on_connect_clicked,
            style_sheet=InstanceWidget.connect_button_stylesheet,
        )

        self.delete_button = misc.pushbutton_maker(
            "Delete",
            constants.ICONS.delete,
            self.on_delete_clicked,
            style_sheet=InstanceWidget.delete_button_stylesheet,
        )
        self.delete_button.setObjectName("deleteButton")

        self.button_layout.addWidget(self.connect_button)
        self.button_layout.addWidget(self.delete_button)
        # endregion buttons

        self.main_layout.addLayout(self.form_layout)
        self.main_layout.addLayout(self.button_layout)
        self.setLayout(self.main_layout)

    def on_instance_name_changed(self) -> None:
        """Handle the instance name change event."""

        new_name = self.instance_name_edit.text().strip()

        if new_name not in ("", self.instance_identifier):
            try:
                constants.REQ_SESSION.post(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["set_instance_name"], f"{self.instance_id}/{new_name}"
                    ),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                self.instance_identifier = new_name
                print(f"[Info] Instance #{self.instance_id} name updated to {self.instance_identifier}.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to update identifier for instance #{self.instance_id}: {e}")

        else:
            self.instance_name_edit.setText(self.instance_identifier)
            print("[Info] Reverted instance name change as it was empty or unchanged.")

    def on_connect_clicked(self) -> None:
        """Handle the connect button click event."""

        if self.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
            print("[Info] Already connected to this instance.")

        else:
            try:
                constants.REQ_SESSION.get(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_instance_info"], str(self.instance_id))
                )

                constants.TELEMETRY_SERVER_INSTANCE_ID = self.instance_id
                constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True
                print(f"[Info] Connected to instance #{self.instance_id} ({self.instance_identifier}).")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to connect to instance #{self.instance_id}: {e}")

    def on_delete_clicked(self) -> None:
        """Handle the delete button click event."""

        if self.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
            try:
                new_instance_id: int = constants.REQ_SESSION.get(
                    constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()
                constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id
                constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True

                constants.REQ_SESSION.delete(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["delete_instance"], str(self.instance_id)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )

                print(
                    f"[Info] Instance #{self.instance_id} deleted and new instance created with ID #{constants.TELEMETRY_SERVER_INSTANCE_ID}."
                )

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to delete instance #{self.instance_id}: {e}")

        else:
            try:
                constants.REQ_SESSION.delete(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["delete_instance"], str(self.instance_id)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                print(f"[Info] Instance #{self.instance_id} deleted successfully.")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to delete instance #{self.instance_id}: {e}")
