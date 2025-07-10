import constants
from syntax_highlighters.json import JsonHighlighter
from widgets.popup_edit import TextEditWindow
from copy import deepcopy
import json
from jsonc_parser.parser import JsoncParser
from qtpy.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QLineEdit,
    QSpacerItem,
    QSizePolicy,
    QScrollArea,
    QPushButton,
    QFrame,
)
from qtpy.QtCore import Qt


class AutopilotParamWidget(QFrame):
    """
    A widget for displaying autopilot parameters and interacting with them.

    Parameters
    ----------
    name
        The name of the parameter.

    default_val
        The default value of the parameter.

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
            assert isinstance(self.default_val, self.type), (
                f"Default value must be of type {self.type.__name__}."
            )
            assert isinstance(self.description, str), "Description must be a string."

        except (KeyError, IndexError):
            raise ValueError(
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
            lambda: None,
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
        self.right_layout.addWidget(self.reset_button)
        # endregion right layout

        self.main_layout.addLayout(self.left_layout, 70)  # 70% width
        self.main_layout.addLayout(self.right_layout, 30)  # 30% width

        self.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.setLineWidth(1)

    def reset_value(self) -> None:
        """Reset the value of the parameter to its default value."""

        self.value = deepcopy(self.default_val)
        if isinstance(self.modify_element, QLineEdit):
            self.modify_element.setText(str(self.value))
        elif self.value_display:
            self.value_display.setText(str(self.value))
            print(f"Info: {self.name} reset to default value: {self.value}.")

        self.reset_button.setEnabled(False)

    def update_value_from_lineedit(self) -> None:
        """Update value from `QLineEdit` input."""

        try:
            text = self.modify_element.text()
            if self.type is bool:
                # Handle boolean conversion
                self.value = text.lower() in ("true", "1", "yes", "on")
            else:
                self.value = self.type(text)
        except ValueError:
            print(f"Error: Invalid value for {self.name}. Resetting to previous value.")
            self.modify_element.setText(str(self.value))

        self.send_button.setEnabled(True)
        self.reset_button.setEnabled(True)

    def edit_sequence_data(self) -> None:
        """Open a text editor for editing a sequence of values."""

        try:
            initial_text = json.dumps(self.value, indent=2)
            self.text_edit_window = TextEditWindow(
                highlighter=JsonHighlighter, initial_text=initial_text
            )
            self.text_edit_window.setWindowTitle(f"Edit {self.name}")
            self.text_edit_window.user_text_emitter.connect(
                self.edit_sequence_data_callback
            )
            self.text_edit_window.show()

        except Exception as e:
            print(f"Error: Failed to open text edit window for {self.name}: {e}")

    def edit_sequence_data_callback(self, text: str) -> None:
        """Callback function for the `edit_sequence_data` function."""

        try:
            edited_data = json.loads(text)

            if self.type is tuple:
                edited_data = tuple(edited_data)
            elif self.type is set:
                edited_data = set(edited_data)

            if not isinstance(edited_data, self.type):
                raise TypeError(f"Edited data must be of type {self.type.__name__}.")

            if edited_data == self.value:
                print(f"Info: No changes made to {self.name}.")
            else:
                self.value = edited_data
                if self.value_display:
                    self.value_display.setText(str(self.value))
                print(f"Info: {self.name} updated to {self.value}.")

        except (json.JSONDecodeError, TypeError, ValueError) as e:
            print(f"Error: Invalid data format for {self.name}: {e}")

        self.send_button.setEnabled(True)
        self.reset_button.setEnabled(True)


class AutopilotParamEditor(QWidget):
    """
    A widget for interacting with and editing autopilot parameters.

    Inherits
    --------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        try:
            self.config: dict = JsoncParser.parse_file(
                constants.AUTO_PILOT_PARAMS_DIR / "params_default.jsonc"
            )
            print(
                f"Info: Loaded {len(self.config)} parameters from `{constants.AUTO_PILOT_PARAMS_DIR / 'params_default.jsonc'}`."
            )
        except Exception as e:
            print(
                f"Error loading autopilot parameters: {e}\n"
                "Please ensure the file exists in the `app_data/autopilot_params` directory."
            )
            self.config = {}

        self.params_container = QWidget()
        self.params_layout = QVBoxLayout()
        self.params_layout.setAlignment(Qt.AlignTop)
        self.params_container.setLayout(self.params_layout)
        self.widgets = []

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

        self.main_layout.addWidget(self.searchbar)
        self.main_layout.addWidget(self.status_label)
        self.main_layout.addWidget(self.scroll)

        self.add_parameters()
        self.update_status_label()

    def add_parameters(self) -> None:
        """Add all parameters to the layout."""

        # Clear existing widgets
        for widget in self.widgets:
            widget.deleteLater()
        self.widgets = []

        for key in self.config.keys():
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

    def update_status_label(
        self, visible_count: int = None, search_text: str = ""
    ) -> None:
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
        else:
            if visible_count == 0:
                self.status_label.setText(f"No parameters match '{search_text}'")
            else:
                self.status_label.setText(
                    f"Showing {visible_count} parameters matching '{search_text}'"
                )
