import requests
import random
from utils import constants, thread_classes, misc
from urllib.parse import urljoin
from datetime import datetime, timezone
from typing import Any
from collections.abc import Callable
from enum import auto
from strenum import StrEnum
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QApplication,
    QWidget,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QScrollArea,
    QLineEdit,
    QLabel,
    QFormLayout,
    QMessageBox,
    QGroupBox,
    QComboBox,
)


class InstanceInfo:
    """
    A class to represent instance information.

    Parameters
    ----------
    data
        A dictionary containing instance information with the following keys:
        - `instance_id`: The unique identifier for the instance.
        - `instance_identifier`: A human-readable name for the instance.
        - `user`: The user associated with the instance.
        - `created_at`: The timestamp when the instance was created (ISO format).
        - `updated_at`: The timestamp when the instance was last updated (ISO format).

    Raises
    ------
    ValueError
        If the `data` dictionary does not contain the required fields or if they are of incorrect types.
    """

    def __init__(self, data: dict[str, Any]) -> None:
        try:
            self.instance_id = int(data["instance_id"])
            self.instance_identifier = str(data["instance_identifier"])
            self.user = str(data["user"])
            self.created_at = self.utc_to_local(datetime.fromisoformat(data["created_at"]))
            self.updated_at = self.utc_to_local(datetime.fromisoformat(data["updated_at"]))

        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"Invalid instance_info data: {e}")

    def as_dict(self) -> dict[str, Any]:
        """
        Return the instance information as a dictionary.

        Returns
        -------
        dict[str, Any]
            A dictionary containing the instance information.
        """

        return {
            "instance_id": self.instance_id,
            "instance_identifier": self.instance_identifier,
            "user": self.user,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
        }

    def utc_to_local(self, utc_dt: datetime) -> datetime:
        """
        Convert a UTC datetime to local timezone.

        Parameters
        ----------
        utc_dt : datetime
            The UTC datetime to convert.

        Returns
        -------
        datetime
            The converted local datetime.
        """

        if utc_dt.tzinfo is None:
            utc_dt = utc_dt.replace(tzinfo=timezone.utc)

        return utc_dt.astimezone()


class InstanceHandler(QWidget):
    """
    This widget is responsible for providing a interface that allows users to \\
    select a instance of the simulation to connect to. Additionally, this interface \\
    will allow users to manage instances that are no longer running but still available.

    Inherits
    --------
    `QWidget`
    """

    class SortBy(StrEnum):
        """
        Enum representing the options for sorting instances.

        Attributes
        ----------
        TIME_SINCE_UPDATED
            Sort by time since last update.
        INSTANCE_ID
            Sort by instance ID.
        INSTANCE_NAME
            Sort by instance name.
        CREATED_AT
            Sort by creation timestamp.
        NO_SORT
            No sorting applied.

        Inherits
        --------
        `StrEnum`
        """

        TIME_SINCE_UPDATED = auto()
        INSTANCE_ID = auto()
        INSTANCE_NAME = auto()
        CREATED_AT = auto()
        NO_SORT = auto()

    def __init__(self) -> None:
        super().__init__()

        self.timer = misc.copy_qtimer(constants.ONE_SECOND_TIMER)
        self.widgets_by_id: dict[int, InstanceWidget] = {}

        self.sort_by: InstanceHandler.SortBy = InstanceHandler.SortBy.TIME_SINCE_UPDATED
        self.on_sort_by_changed(self.sort_by)

        self.current_search_text: str = ""

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

        sort_layout = QHBoxLayout()
        sort_layout.setContentsMargins(0, 0, 0, 0)
        sort_layout.setSpacing(5)

        self.sort_label = QLabel("Sort by:")
        self.sort_by_dropdown = QComboBox()
        self.sort_label.setBuddy(self.sort_by_dropdown)
        self.sort_by_dropdown.addItems([option.value for option in InstanceHandler.SortBy])
        self.sort_by_dropdown.setCurrentText(self.sort_by.value)
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

        self.main_layout.addLayout(sort_layout, 0, 0)
        self.main_layout.addWidget(self.searchbar, 2, 0)
        self.main_layout.addWidget(self.status_label, 3, 0)
        self.main_layout.addWidget(self.scroll, 4, 0)
        self.main_layout.addWidget(self.button_groupbox, 5, 0)

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

        # region update and create new instance widgets
        # note that new_ids is not necessarily a superset of existing_ids
        existing_ids = set(self.widgets_by_id.keys())
        new_ids: set[int] = {
            instance["instance_id"] for instance in instances if isinstance(instance.get("instance_id"), int)
        }

        # how this works:
        # a = {1, 2, 3, 4, 5}
        # b = {2, 3, 4, 5, 6}
        # a - b = {1} and b - a = {6}
        deprecated_ids = existing_ids - new_ids
        not_deprecated_ids = list(new_ids - deprecated_ids)

        for instance_id in deprecated_ids:
            widget = self.widgets_by_id.pop(instance_id)
            self.instances_layout.removeWidget(widget)
            widget.deleteLater()

            if instance_id == constants.TELEMETRY_SERVER_INSTANCE_ID:
                if len(not_deprecated_ids) >= 1:
                    constants.TELEMETRY_SERVER_INSTANCE_ID = random.choice(not_deprecated_ids)
                    constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True
                    print(
                        f"The instance you were connected to, #{instance_id}, has been removed. You have been connected to instance #{constants.TELEMETRY_SERVER_INSTANCE_ID} instead. Please select a different instance if needed."
                    )

                else:
                    try:
                        new_instance_id: int = constants.REQ_SESSION.get(
                            constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                            timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                        ).json()
                        constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id
                        constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True
                        print(
                            f"The instance you were connected to, #{instance_id}, has been removed. A new instance has been created with ID #{new_instance_id} and you have been connected to it."
                        )

                    except requests.exceptions.RequestException as e:
                        print(f"[Error] Failed to create a new instance, exiting application. Error: {e}")
                        QApplication.quit()

        for instance in instances:
            try:
                instance_info = InstanceInfo(instance)

            except ValueError as e:
                print(f"[Warning] Skipping invalid instance data: {e}")
                continue

            # checking if we have seen this instance before
            widget = self.widgets_by_id.get(instance_info.instance_id)
            if widget:
                widget.updated_at = instance_info.updated_at
                widget.updated_at_label.setText(instance_info.updated_at.strftime("%Y-%m-%d %I:%M:%S %p"))

                if widget.instance_identifier != instance_info.instance_identifier:
                    widget.instance_name_edit.setText(instance_info.instance_identifier)

            else:
                try:
                    self.widgets_by_id[instance_info.instance_id] = InstanceWidget(instance)

                except ValueError as e:
                    print(f"[Warning] Skipping invalid instance data: {e}")

        for widget in sorted(self.widgets_by_id.values(), key=self.sort_key):
            self.instances_layout.addWidget(widget)
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
            alert_message = "All instances deleted successfully."

            if constants.TELEMETRY_SERVER_INSTANCE_ID != -1:
                new_instance_id: int = constants.REQ_SESSION.get(
                    constants.TELEMETRY_SERVER_ENDPOINTS["create_instance"],
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                ).json()

                alert_message = f"All instances deleted. New instance created with ID #{new_instance_id}."
                constants.TELEMETRY_SERVER_INSTANCE_ID = new_instance_id
                constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = True

            print(f"[Info] {alert_message}")

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to delete all instances: {e}")

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

    def on_sort_by_changed(self, sort_method: "InstanceHandler.SortBy") -> None:
        """
        Handle the sort by dropdown change event.

        Parameters
        ----------
        sort_method
            The selected sort method.
        """

        try:
            self.sort_by = InstanceHandler.SortBy(sort_method)

            if self.sort_by == InstanceHandler.SortBy.INSTANCE_ID:
                self.sort_key: Callable[[InstanceWidget], int] = lambda w: w.instance_id

            elif self.sort_by == InstanceHandler.SortBy.INSTANCE_NAME:
                self.sort_key: Callable[[InstanceWidget], str] = lambda w: w.instance_identifier.lower()

            elif self.sort_by == InstanceHandler.SortBy.CREATED_AT:
                self.sort_key: Callable[[InstanceWidget], datetime] = lambda w: w.created_at

            elif self.sort_by == InstanceHandler.SortBy.TIME_SINCE_UPDATED:
                self.sort_key: Callable[[InstanceWidget], float] = lambda w: (
                    (datetime.now().astimezone() - w.updated_at).total_seconds()
                    if (w.updated_at - w.created_at).total_seconds() > 1.0
                    else float("inf")
                )

            elif self.sort_by == InstanceHandler.SortBy.NO_SORT:
                self.sort_key: Callable[[InstanceWidget], int] = lambda w: 0

        except ValueError:
            print(f"[Warning] Unknown sort method selected: {sort_method}. Defaulting to INSTANCE_ID.")
            self.sort_by = InstanceHandler.SortBy.INSTANCE_ID

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
    This widget is responsible for displaying the information of a single instance \\
    and providing controls to connect or disconnect from it.

    Parameters
    ----------
    instance_info
        A dictionary containing information about the instance. It should include (at least):
        - `instance_id`: The unique identifier for the instance.
        - `instance_identifier`: A human-readable name for the instance.
        - `user`: The user associated with the instance.
        - `created_at`: The timestamp when the instance was created (ISO format).
        - `updated_at`: The timestamp when the instance was last updated (ISO format).

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

        self.instance_info = InstanceInfo(instance_info)

        self.instance_id = self.instance_info.instance_id
        self.instance_identifier = self.instance_info.instance_identifier
        self.user = self.instance_info.user
        self.created_at = self.instance_info.created_at
        self.updated_at = self.instance_info.updated_at

        self.main_layout = QHBoxLayout()
        self.main_layout.setContentsMargins(10, 10, 10, 10)

        # region setup widget style
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setStyleSheet(InstanceWidget.style_sheet)

        self.form_layout = QFormLayout()
        self.instance_name_edit = QLineEdit(self.instance_identifier)
        self.instance_name_edit.editingFinished.connect(self.on_instance_name_changed)
        self.user_label = QLabel(self.user)
        self.instance_id_label = QLabel(str(self.instance_id))
        self.created_at_label = QLabel(self.created_at.strftime("%Y-%m-%d %I:%M:%S %p"))
        self.updated_at_label = QLabel(self.updated_at.strftime("%Y-%m-%d %I:%M:%S %p"))

        self.form_layout.addRow("Instance Name", self.instance_name_edit)
        self.form_layout.addRow("User", self.user_label)
        self.form_layout.addRow("Instance ID", self.instance_id_label)
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
