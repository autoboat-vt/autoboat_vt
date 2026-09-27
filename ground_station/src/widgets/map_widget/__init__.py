"""
Package for the map widget in the Groundstation application.

Exposes:
- :module:`callback_server`: A subpackage that contains the callback server for handling map-related requests.
- :class:`MapBridge`: A class which provides a typed Python interface to the Typescript `MapInterface`.
- :class:`MapOptionsHandler`: A class that manages the map options and features.
- :class:`LandClickPrompt`: A class that handles user prompts for land click confirmations.
- :data:`LAND_CLICK_PROMPT`: A singleton instance of :class:`LandClickPrompt` that manages the land click prompt state.

Contains:
- `bridge.py`: Contains the :class:`MapBridge` class which provides a typed Python interface to the Typescript `MapInterface`.
- `map_options_handler.py`: Contains the :class:`MapOptionsHandler` class for managing map options and features.
- `land_click_prompt.py`: Contains the :class:`LandClickPrompt` class that handles user prompts for land click confirmations.
- `frontend`: A directory containing the HTML and Typescript code for the map widget's frontend interface.
- `callback_server`: A subpackage that contains the callback server for handling map-related requests.
"""

__all__ = ["LAND_CLICK_PROMPT", "LandClickPrompt", "MapBridge", "MapOptionsHandler", "callback_server"]

from . import callback_server
from .bridge import MapBridge
from .land_click_prompt import LAND_CLICK_PROMPT, LandClickPrompt
from .map_options_handler import MapOptionsHandler
