"""
Package for the map widget in the Groundstation application.

Exposes:
- :module:`callback_server`: A subpackage that contains the callback server for handling map-related requests.
- :class:`MapViewerServer`: A class that manages the map viewer server process and the page that renders its
  frontend.
- :class:`MapViewerServer.TypeScriptInterface`: A class which provides a typed Python interface to the Typescript
  `MapInterface`.
- :class:`MapViewerServer.TypeScriptInterface.BrowserLogger`: A :class:`QWebEnginePage` subclass that forwards the
  frontend's JavaScript console output to the Qt logger.
- :class:`MapOptionsHandler`: A class that manages the map options and features.
- :class:`LandClickPrompt`: A class that handles user prompts for land click confirmations.
- :data:`LAND_CLICK_PROMPT`: A singleton instance of :class:`LandClickPrompt` that manages the land click prompt state.

Contains:
- `bridge.py`: Contains the :class:`MapViewerServer` class that manages the map viewer server process, its nested
  :class:`MapViewerServer.TypeScriptInterface` bridge to the Typescript `MapInterface`, and its nested
  :class:`MapViewerServer.TypeScriptInterface.BrowserLogger` page which forwards the frontend's JavaScript console
  output to the Qt logger.
- `map_options_handler.py`: Contains the :class:`MapOptionsHandler` class for managing map options and features.
- `land_click_prompt.py`: Contains the :class:`LandClickPrompt` class that handles user prompts for land click confirmations.
- `frontend`: A directory containing the HTML and Typescript code for the map widget's frontend interface.
- `callback_server`: A subpackage that contains the callback server for handling map-related requests.
"""

__all__ = [
    "LAND_CLICK_PROMPT",
    "LandClickPrompt",
    "MapOptionsHandler",
    "MapViewerServer",
    "callback_server",
]

from . import callback_server
from .bridge import MapViewerServer
from .land_click_prompt import LAND_CLICK_PROMPT, LandClickPrompt
from .map_options_handler import MapOptionsHandler
