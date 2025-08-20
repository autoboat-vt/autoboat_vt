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

        self.instance_widgets: list[InstanceWidget] = []
        self.instance_widgets_by_id: dict[int, InstanceWidget] = {}
        self.instance_info_by_id: dict[int, dict[str, Any]] = {}

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

        self.timer.start()

    def update_instances_starter(self) -> None:
        """Start the instance fetcher thread."""

        if not self.instance_fetcher.isRunning():
            self.instance_fetcher.start()

    def update_instances(self, instances: list[int]) -> None:
        """
        Update the instance widgets based on the fetched instances.

        Parameters
        ----------
        instances
            A list of instance IDs fetched from the telemetry server.
        """

        new_ids = list(instances)  # keep order from server

        new_id_set = set(new_ids)
        old_id_set = set(self.instance_widgets_by_id.keys())

        # Avoid repaint churn while we rearrange
        self.instances_container.setUpdatesEnabled(False)

        # 1) Remove widgets that disappeared
        for removed_id in old_id_set - new_id_set:
            w = self.instance_widgets_by_id.pop(removed_id, None)
            if w is not None:
                self.instances_layout.removeWidget(w)
                w.deleteLater()
            self.instance_info_by_id.pop(removed_id, None)

        # 2) Fetch info for all current IDs and create/update widgets
        for iid in new_ids:
            try:
                info = constants.REQ_SESSION.get(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_instance_info"], str(iid)),
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()
            except requests.exceptions.RequestException as e:
                print(f"[Warning] Failed to fetch instance info for {iid}: {e}")
                # If fetch fails and widget already exists, keep the old UI; if not, skip.
                continue

            self.instance_info_by_id[iid] = info

            if iid in self.instance_widgets_by_id:
                # Update existing widget in-place (no flicker)
                self.instance_widgets_by_id[iid].update_from_info(info)
            else:
                # Create new widget
                w = InstanceWidget(info)
                self.instance_widgets_by_id[iid] = w
                self.instances_layout.addWidget(w)

        # 3) Reorder widgets to match server order (cheap remove+add without destroying)
        for iid in new_ids:
            w = self.instance_widgets_by_id.get(iid)
            if w is not None:
                self.instances_layout.removeWidget(w)
                self.instances_layout.addWidget(w)

        # Keep a list version for filtering convenience (preserve order)
        self.instance_widgets = [self.instance_widgets_by_id[iid] for iid in new_ids if iid in self.instance_widgets_by_id]

        # 4) Re-apply search or update status label
        if self.current_search_text:
            self.filter_instances(self.current_search_text)
        elif self.instance_widgets:
            self.status_label.setText(f"{len(self.instance_widgets)} instances found")
        else:
            self.status_label.setText("No instances found")

        self.instances_container.setUpdatesEnabled(True)
        self.instances_container.update()

        if constants.TELEMETRY_SERVER_INSTANCE_ID == -1:
            self.timer.stop()

            if len(new_ids) == 1:
                constants.TELEMETRY_SERVER_INSTANCE_ID = new_ids[0]
                print(f"[Info] Automatically connected to instance {new_ids[0]} as the only available instance.")

            else:
                main_window = self.window()
                tab_widget = main_window.centralWidget()
                if isinstance(tab_widget, QTabWidget):
                    tab_widget.setCurrentWidget(self)

                while True:
                    new_id = constants.show_input_dialog(
                        "Select Instance",
                        "Please select the instance you want to connect to:",
                        input_type=int,
                    )

                    if new_id is not None and new_id in new_ids:
                        constants.TELEMETRY_SERVER_INSTANCE_ID = new_id
                        print(f"[Info] Connected to instance {new_id}")
                        break

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

        for widget in self.instance_widgets:
            if text.lower() in widget.instance_identifier.lower():
                widget.show()
            else:
                widget.hide()

        visible_count = sum(widget.isVisible() for widget in self.instance_widgets)
        if text:
            self.status_label.setText(f"{visible_count} instances found matching '{text}'")
        else:
            self.status_label.setText(f"{len(self.instance_widgets)} instances found")


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

        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setStyleSheet("""
            QFrame { background-color: #2E2E2E; border-radius: 0px; border: 1px solid #444; }
            QLabel { color: white; font-size: 12pt; }
            QLineEdit { background-color: #3E3E3E; color: white; border: 1px solid #555; border-radius: 3px; padding: 5px; }
            QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px 12px; border-radius: 5px; }
            QPushButton:hover { background-color: #45A049; }
            QPushButton#deleteButton { background-color: #F44336; }
            QPushButton#deleteButton:hover { background-color: #D32F2F; }
        """)

        self.main_layout = QHBoxLayout()
        self.setLayout(self.main_layout)
        self.main_layout.setContentsMargins(10, 10, 10, 10)

        # Form layout for instance info
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

        # Buttons
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

        self.main_layout.addLayout(self.form_layout)
        self.main_layout.addLayout(self.button_layout)

    def update_from_info(self, instance_info: dict) -> None:
        """Update labels/fields without recreating the widget."""

        try:
            # ID should be stable; keep a sanity check but don't overwrite.
            if int(instance_info["instance_id"]) != self.instance_id:
                print(
                    f"[Warn] Instance ID changed for widget (old {self.instance_id} != new {instance_info['instance_id']}). Ignoring ID change."
                )
            new_identifier = instance_info["instance_identifier"]
            new_created_at = datetime.fromisoformat(instance_info["created_at"])
            new_updated_at = datetime.fromisoformat(instance_info["updated_at"])

        except Exception as e:
            print(f"[Warn] Bad instance_info during update: {e}")
            return

        # Update identifier (affects filtering)
        if new_identifier != self.instance_identifier:
            self.instance_identifier = new_identifier
            # Only update the edit if the user isn't actively editing
            if not self.instance_name_edit.isModified():
                self.instance_name_edit.setText(new_identifier)

        # Update timestamps if changed
        if new_created_at != getattr(self, "created_at", None):
            self.created_at = new_created_at
            self.created_at_label.setText(self.created_at.strftime("%Y-%m-%d %H:%M:%S"))

        if new_updated_at != getattr(self, "updated_at", None):
            self.updated_at = new_updated_at
            self.updated_at_label.setText(self.updated_at.strftime("%Y-%m-%d %H:%M:%S"))

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
            return

        try:
            constants.TELEMETRY_SERVER_INSTANCE_ID = self.instance_id
            print(f"[Info] Connected to instance {self.instance_identifier} ({self.instance_id}).")

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to connect to instance {self.instance_identifier}: {e}")

    def on_delete_clicked(self) -> None:
        """Handle the delete button click event."""

        if self.instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
            new_instance_id = constants.REQ_SESSION.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()["id"]
            constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id

            constants.REQ_SESSION.delete(
                urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["delete_instance"], str(self.instance_id)),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            )
            print(
                f"[Info] Instance {self.instance_id} deleted and new instance created with ID {constants.TELEMETRY_SERVER_INSTANCE_ID}."
            )
            return

        try:
            constants.REQ_SESSION.delete(
                urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["delete_instance"], str(self.instance_id)),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            )
            print(f"[Info] Instance {self.instance_id} deleted successfully.")

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to delete instance {self.instance_id}: {e}")
