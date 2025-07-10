import constants
from syntax_highlighters.json import JsonHighlighter
from widgets.popup_edit import TextEditWindow
from copy import deepcopy

import ast
from jsonc_parser.parser import JsoncParser
from PyQt5.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QSpacerItem,
    QSizePolicy,
    QScrollArea,
)
from PyQt5.QtCore import Qt


class AutopilotParamWidget(QWidget):
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
    `QWidget`
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
            self.name: str = config.keys()[0]
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

        self.layout = QGridLayout()

        # for button name and interface to control it
        self.left_layout = QHBoxLayout()

        # for the send and reset buttons
        # top is send, bottom is reset
        self.right_layout = QVBoxLayout()
        # endregion define layouts

        # region left layout
        self.label = QLabel(self.name)
        self.label.setToolTip(self.description)

        if isinstance(self.type, (list, dict, tuple, set)):
            self.modify_element = constants.pushbutton_maker(
                self.name, constants.ICONS.pencil, self.edit_sequence_data
            )

        else:
            self.modify_element = QLineEdit(self.value)

        self.label.setBuddy(self.modify_element)

        self.left_layout.addWidget(self.label)
        self.left_layout.addWidget(self.modify_element)
        # endregion left layout
        # region right layout
        self.send_button = constants.pushbutton_maker(
            "Send",
            constants.ICONS.upload,
            lambda: None,
            max_width=100,
            min_height=30,
            is_clickable=False,
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

    def reset_value(self) -> None:
        """
        Reset the value of the parameter to its default value.
        """

        self.value = self.default_val
        if isinstance(self.modify_element, QLineEdit):
            self.modify_element.setText(str(self.value))
        else:
            print(f"Info: {self.name} reset to default value: {self.value}.")

    def edit_sequence_data(self) -> None:
        """
        Open a text editor for editing a sequence of values.

        `self.edit_sequence_callback` is called when the user closes or clicks the save button in the text edit window.
        `self.edit_sequence_callback` recieves the text from the text edit window when the user clicks the save button,
        otherwise it recieves the text without any changes.
        """

        try:
            initial_text = str(self.value)
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
        """
        Callback function for the `edit_sequence_data` function.

        This function is called when the user closes the text edit window.
        It retrieves the edited text and saves it to the `self.buoys` variable and closes the window.

        Parameters
        ----------
        text
            The text entered by the user in the text edit window.
        """

        try:
            edited_data = ast.literal_eval(text)
            if edited_data == self.value:
                print(f"Info: No changes made to {self.name}.")
            else:
                if isinstance(edited_data, self.type):
                    self.value = edited_data
                    print(f"Info: {self.name} updated to {self.value}.")
                else:
                    raise TypeError(
                        f"Error: Edited data must be of type {self.type.__name__}."
                    )
        except (SyntaxError, ValueError) as e:
            print(f"Error: Invalid data format for {self.name}: {e}")


class AutopilotParamEditor(QWidget):
    """
    A layout for displaying autopilot parameters in a grid format.

    Inherits
    --------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()
        self.config: dict = JsoncParser.parse_file(
            constants.AUTO_PILOT_PARAMS_DIR / "editor_config.jsonc"
        )

        self.scroll = QScrollArea()
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidgetResizable(True)

        self.params_widget = QWidget()
        self.params_layout = QVBoxLayout()
        self.params.setLayout(self.params_layout)
        self.widgets = []

        for key in self.config.keys():
            param_widget = AutopilotParamWidget(self.config[key])
            self.params_layout.addWidget(param_widget)
            self.widgets.append(param_widget)

        spacer = QSpacerItem(1, 1, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.params_layout.addItem(spacer)

        # Scroll Area Properties.
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.params_widget)

        # Search bar.
        self.searchbar = QLineEdit()

        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.searchbar)
        main_layout.addWidget(self.scroll)
