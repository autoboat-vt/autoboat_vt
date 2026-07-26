---
description: "Use when writing or editing the PyQt ground station or its Vite/TS map widget. Covers StateManager singleton + STATE_FILE_CONTENTS defaults, GroundStationWidget 3-section layout + QWebEngineView singleton MAP_PAGE + 2 timers, AutopilotConfigWidget/Editor/ParamWidget/Manager (content-addressed by hash), CameraWidget (QWebEngineView + JS setBase64Image), InstanceHandler (connection_history deque + ID reuse detection), widgets/__init__.py registration pattern, dialog_templates BaseDialog, map_widget Python ThreadingHTTPServer on MAP_SERVER_PORT 3002 (NOT Vite), waypoints_handler CORS GET/POST, GraphViewer pyqtgraph source, run.sh (bun run serve on VITE_PORT 5173 + python src/main.py)."
applyTo: "ground_station/**"
---

# Ground Station Conventions

The ground station is a **PyQt desktop app** (PySide6 via `qtpy`) that embeds a **Vite/TypeScript + Leaflet map widget** in a `QWebEngineView`. It also runs a small **Python HTTP server** for waypoint exchange with the map widget. See `typescript.instructions.md` for the map frontend internals.

## Launching (host with display only)

```bash
cd ground_station && ./run.sh
```

`run.sh`:
- Starts the Vite map server: `bun run serve` on `VITE_PORT` (default `5173`), `127.0.0.1`.
- Launches the PyQt app: `python src/main.py`.

> ⚠️ **The map widget is served by Vite, NOT by `map_widget/server.py`.** `server.py` is a separate Python `ThreadingHTTPServer` on `MAP_SERVER_PORT` (default `3002`) that the map widget's JS calls to GET/POST waypoints (CORS-enabled). These are two different servers on two different ports — don't conflate them.

## Persistent state — `StateManager` singleton

File: `ground_station/src/utils/state_manager.py`. Singleton exposed as `constants.SM` (instance) and `constants.SM_class` (class). Backed by `app_data/git_ignore/app_state.json` (gitignored).

**Rule:** add new keys to `constants.STATE_FILE_CONTENTS` BEFORE reading them — `StateManager.read` returns the default from `STATE_FILE_CONTENTS` for unknown keys but doesn't persist them unless the file was seeded with them.

```python
# constants.py — define defaults here
STATE_FILE_CONTENTS: dict[str, Any] = {
    "ip_addresses": {"port": 3000},
    "selected_boat_instance_id": None,
    "autopilot_param_filters": ["position"],
    "boat_instances": [],                       # ⚠️ not in STATE_FILE_CONTENTS originally — drift
    "keybindings": { ... },                     # ⚠️ not in STATE_FILE_CONTENTS originally — drift
    # add new keys here before reading
}
SM = StateManager(STATE_FILE_CONTENTS)
```

### Typed read API

```python
SM.write("key.path", value)                    # creates nested dicts as needed
SM.read("key.path", default=...)               # untyped
SM.read_str("key.path", default="...")
SM.read_int("key.path", default=0)
SM.read_float("key.path", default=0.0)
SM.read_bool("key.path", default=False)
SM.read_list("key.path", default=[])           # always returns a list
SM.read_dict("key.path", default={})           # always returns a dict
SM.save()                                      # flush to disk
```

> ⚠️ Known drift: `boat_instances` and `keybindings` are read by code but missing from `STATE_FILE_CONTENTS` — they work via fallback defaults but won't persist on a fresh install until added to the dict.

## `GroundStationWidget` — top-level layout

File: `ground_station/src/widgets/ground_station_widget.py`. Three-section layout, hosts the `QWebEngineView` running the map, two `QTimer`s for periodic work, keybind manager, `eventFilter` for `Ctrl+Z` undo.

```python
from qtpy.QtWidgets import QHBoxLayout, QSplitter, QVBoxLayout, QWidget
from qtpy.QtCore import QTimer
from qtpy.QtWebEngineWidgets import QWebEngineView

from utils import constants
from utils.state_manager import StateManager
from widgets.autopilot_config_widget import AutopilotConfigWidget
from widgets.camera_widget import CameraWidget
from widgets.map_widget.map_widget import MapWidget
from widgets.graph_viewer import GraphViewer
from widgets.instance_handler import InstanceHandler


class GroundStationWidget(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._setup_ui()
        self._setup_timers()
        self._setup_keybinds()
        self.installEventFilter(self)

    def _setup_ui(self) -> None:
        # 3-section: left (camera + instances), center (map), right (autopilot config)
        main_layout = QHBoxLayout(self)
        splitter = QSplitter()
        main_layout.addWidget(splitter)

        self.camera_widget = CameraWidget()
        self.instance_handler = InstanceHandler()
        left_panel = QWidget(); left_layout = QVBoxLayout(left_panel)
        left_layout.addWidget(self.camera_widget)
        left_layout.addWidget(self.instance_handler)

        self.map_widget = MapWidget(constants.MAP_PAGE)   # singleton URL, see below
        self.autopilot_config_widget = AutopilotConfigWidget()

        splitter.addWidget(left_panel)
        splitter.addWidget(self.map_widget)
        splitter.addWidget(self.autopilot_config_widget)

    def _setup_timers(self) -> None:
        self.fast_timer = QTimer(self)                   # ~50ms — UI refresh
        self.fast_timer.timeout.connect(self._fast_tick)
        self.fast_timer.start(50)
        self.slow_timer = QTimer(self)                   # ~1s — poll boats
        self.slow_timer.timeout.connect(self._slow_tick)
        self.slow_timer.start(1000)
```

### `MAP_PAGE` singleton

`constants.MAP_PAGE` is a single `QUrl` constructed once (from `http://127.0.0.1:{VITE_PORT}`) and reused — only one `QWebEngineView` instance across the app. Pass it to `MapWidget`:

```python
self.map_widget = MapWidget(constants.MAP_PAGE)
```

### `eventFilter` for `Ctrl+Z`

```python
def eventFilter(self, obj, event):
    if event.type() == QEvent.Type.KeyPress:
        if event.key() == Qt.Key.Key_Z and (event.modifiers() & Qt.KeyboardModifier.ControlModifier):
            self._undo_last_action()
            return True
    return super().eventFilter(obj, event)
```

## `AutopilotConfigWidget` — tabbed editor

File: `ground_station/src/widgets/autopilot_config/`. Composed of:
- `AutopilotConfigWidget` — outer container, `QTabWidget` in `South` position.
- `AutopilotConfigEditor` — loads defaults from telemetry server, renders one `AutopilotParamWidget` per JSON entry, exposes `refresh_signal`, `send_all`, `pull_all`, `clear`, `load`, `save`, `filter`.
- `AutopilotParamWidget` — single param row, methods: `send_value`, `pull_value`, `reset`, `edit_grouped_data` (for list-type params → opens `TextEditWindow`).
- `ConfigInfo` + `AutopilotConfigManager` — content-addressed by hash (the JSON string is hashed; configs cached by hash).

### `AutopilotConfigEditor` API surface

```python
class AutopilotConfigEditor(QWidget):
    refresh_signal = Signal()

    def __init__(self, instance: BoatInstance, parent=None): ...
    def send_all(self) -> None: ...      # POST all params to boat's telemetry server
    def pull_all(self) -> None: ...      # GET params from boat's telemetry server
    def clear(self) -> None: ...         # reset UI to defaults
    def load(self) -> None: ...          # load from local file
    def save(self) -> None: ...          # save to local file
    def filter(self, pattern: str) -> None: ...   # filter visible params by name
```

### `AutopilotConfigManager` — content-addressed by hash

```python
class AutopilotConfigManager:
    """Stores configs in memory, keyed by their JSON hash."""
    def __init__(self) -> None:
        self._configs: dict[str, ConfigInfo] = {}     # hash → ConfigInfo

    def add(self, config_json: str) -> ConfigInfo:
        h = hashlib.sha256(config_json.encode()).hexdigest()
        if h not in self._configs:
            self._configs[h] = ConfigInfo(hash=h, json=config_json, ...)
        return self._configs[h]
```

## `CameraWidget` — `QWebEngineView` + base64 image

File: `ground_station/src/widgets/camera_widget.py`. Uses a `QWebEngineView` + `setHtml` to embed a minimal HTML page with an `<img>` tag. Updates the image via `setBase64Image` JS call. Polled by the `HALF_SECOND_TIMER` in `GroundStationWidget` (every 500ms).

```python
class CameraWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.view = QWebEngineView()
        self.view.setHtml('<html><body style="margin:0"><img id="cam" style="width:100%"/></body></html>')
        layout = QVBoxLayout(self); layout.addWidget(self.view)

    def set_image_b64(self, b64_data: str) -> None:
        js = f'setBase64Image("{b64_data}")'
        self.view.page().runJavaScript(js)
```

## `InstanceHandler` — boat connection management

File: `ground_station/src/widgets/instance_handler.py`. Tracks known boat instances (one per IP), lets user select active instance, detects reconnection.

```python
class SortBy(StrEnum):
    IP = "ip"
    NAME = "name"
    LAST_SEEN = "last_seen"

class InstanceHandler(QWidget):
    def __init__(self):
        self.connection_history: deque = deque(maxlen=2)   # last 2 connection IDs
        self.has_telemetry_server_instance_changed: bool = False
        # ... UI ...

    def add_instance(self, instance: BoatInstance) -> None: ...
    def select_instance(self, instance_id: str) -> None: ...
```

**Conventions:**
- `connection_history` is a `deque(maxlen=2)` — rolling last-2 connections.
- **ID reuse detection:** if a new instance appears with the same ID as one already in `connection_history`, it's a reconnection (set `has_telemetry_server_instance_changed = True`).
- `SortBy` is a `StrEnum` — UI sort mode.

## `widgets/__init__.py` — registration pattern

New widgets must be registered in `ground_station/src/widgets/__init__.py` (and `__all__`):

```python
from widgets.camera_widget import CameraWidget
from widgets.ground_station_widget import GroundStationWidget
from widgets.autopilot_config_widget import AutopilotConfigWidget
# add new widgets here

__all__ = [
    "CameraWidget",
    "GroundStationWidget",
    "AutopilotConfigWidget",
    # ... add new widget names here
]
```

## `dialog_templates.py` — `BaseDialog`

File: `ground_station/src/utils/dialog_templates.py`. Base class for all modal dialogs. Has two signals: `user_text_emitter` (emits typed text on confirm) and `dialog_closed` (emits when dialog closes).

```python
class BaseDialog(QDialog):
    user_text_emitter = Signal(str)
    dialog_closed = Signal()

    def __init__(self, title: str, prompt: str, parent=None): ...
    def remember_choice_option(self, key: str) -> None: ...   # persist "don't ask again"
```

`remember_choice_option(key)` persists a "don't ask again" choice to `StateManager` under the `keybindings` key.

## `map_widget/server.py` — waypoint HTTP server (NOT Vite)

File: `ground_station/src/widgets/map_widget/server.py`. Python `ThreadingHTTPServer` on `MAP_SERVER_PORT` (default `3002`). Serves the `/waypoints` endpoint that the map widget's JS calls. Separate from Vite (which serves the map HTML/JS on port 5173).

```python
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from utils import constants

class WaypointsHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):                     # CORS preflight
        self.send_response(204)
        self._send_cors_headers()
    def do_GET(self):                         # GET /waypoints
        with _WAYPOINTS_LOCK:
            data = json.dumps(_WAYPOINTS).encode()
        self.send_response(200)
        self._send_cors_headers()
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(data)
    def do_POST(self):                        # POST /waypoints
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)
        with _WAYPOINTS_LOCK:
            _WAYPOINTS.clear()
            _WAYPOINTS.extend(json.loads(body))
        self.send_response(200); self._send_cors_headers(); self.end_headers()

    def _send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")


_WAYPOINTS: list = []
_WAYPOINTS_LOCK = threading.Lock()

def start_server():
    server = ThreadingHTTPServer(("127.0.0.1", constants.MAP_SERVER_PORT), WaypointsHandler)
    server.serve_forever()
```

### `waypoints_handler.py` — in-memory state + lock

The `_WAYPOINTS` list + `_WAYPOINTS_LOCK` are module-level globals shared between the HTTP handler and the PyQt side (waypoint polling threads read them). The `Lock` is required because the HTTP handler runs in its own thread.

## `GraphViewer` — pyqtgraph telemetry source

File: `ground_station/src/widgets/graph_viewer.py`. Uses `pyqtgraph` to plot live telemetry. Is a **telemetry source** (consumes `/telemetry` data, re-emits as `boat_data_signal`).

```python
from collections import deque
import pyqtgraph as pg
from qtpy.QtCore import Signal

class GraphViewer(QWidget):
    boat_data_signal = Signal(tuple)          # (key, value, timestamp)

    def __init__(self):
        self.important_keys = ["heading", "desired_heading", "rudder_angle", ...]
        self.history_length = 200
        self.data: dict[str, deque] = {k: deque(maxlen=self.history_length) for k in self.important_keys}
        # ... pyqtgraph PlotWidget setup ...

    def add_data(self, key: str, value: float) -> None:
        if key in self.data:
            self.data[key].append((time.time(), value))
            self.boat_data_signal.emit((key, value, time.time()))
```

**Conventions:**
- `history_length = 200` — fixed-length `deque(maxlen=200)` per key.
- `important_keys` — whitelist of telemetry keys to plot.
- `boat_data_signal` is `Signal(tuple)` — emits `(key, value, timestamp)`.
- Other widgets connect to `boat_data_signal` to react to new telemetry.

> ⚠️ **Import `PlotItem` from its full module path, not as `pg.PlotItem`.**
> pyqtgraph exposes `PlotItem` as both a package (`pyqtgraph.graphicsItems.PlotItem/`, a directory) and a class (`pyqtgraph.graphicsItems.PlotItem.PlotItem.PlotItem`, defined in `PlotItem.py` inside that package). At runtime `pg.PlotItem` resolves to the class because `pyqtgraph/__init__.py` re-exports it, but Pylance can't disambiguate the package-vs-class name collision and reports `"Module cannot be used as a type"` on `list[pg.PlotItem]`. Always import the class directly:
> ```python
> import pyqtgraph as pg
> from pyqtgraph.graphicsItems.PlotItem.PlotItem import PlotItem
>
> self.plots: list[PlotItem] = []
> ```
> The class is identical at runtime (`pg.PlotItem is PlotItem` is `True`); this is purely a static-analysis workaround. Use `pg.*` for everything else in pyqtgraph (e.g. `pg.mkPen`, `pg.GraphicsLayoutWidget`).

## Things to avoid

- Editing `app_data/git_ignore/app_state.json` directly — always go through `StateManager`.
- Reading a `StateManager` key without first adding it to `STATE_FILE_CONTENTS` — it'll fall back to a default but won't persist on fresh installs.
- Importing `PyQt5` or `PyQt6` directly — always go through `qtpy` for Qt abstraction.
- Adding new TS files for the map widget outside `ground_station/src/widgets/map_widget/frontend/` — `tsconfig.json` `include` is hard-scoped to that directory.
- Conflating `MAP_SERVER_PORT` (3002, Python `ThreadingHTTPServer` for waypoints) with `VITE_PORT` (5173, Vite dev server for the map HTML/JS) — they're different servers.
- Creating more than one `QWebEngineView` for the map — reuse the `MAP_PAGE` singleton.
- Forgetting the `threading.Lock` around `_WAYPOINTS` — the HTTP handler runs in its own thread.
- Forgetting to register new widgets in `widgets/__init__.py` and add to `__all__`.
- Putting PyQt signal definitions in `__init__` — they must be class attributes.
