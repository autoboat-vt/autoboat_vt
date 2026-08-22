---
description: "Use when writing, editing, or reviewing Python code in this ROS 2 / PyQt ground station workspace. Covers ruff config (select=ALL), strict typing, numpy docstrings, ROS 2 node patterns, StateManager API, MapBridge typed Python→JS bridge (js_load_guard internal), thread_classes Router pattern, and ground_station Qt conventions."
applyTo: "**/*.py"
---

# Python Conventions

All Python is formatted and linted by **ruff** (`ruff.toml` at repo root). Config summary:

- `select = ["ALL"]` with a curated `ignore` list — do not add new ignores without reason.
- `line-length = 130`, `indent-width = 4`.
- `future-annotations = true` → **always** start modules with `from __future__ import annotations`.
- isort section order: `future, standard-library, third-party, qtpy, ros, first-party, local-folder`. `qtpy` and `ros` are custom sections — see `ruff.toml` for which modules belong where.
- `known-first-party` includes all ROS package names (`autoboat_msgs`, `autopilot`, `drivers`, `object_detection`, `simulation_*`, etc.) and `ground_station`, `utils`, `widgets`, `map_widget`.
- pydocstyle convention = `numpy`.
- `unfixable = ["F401"]` — unused imports are flagged but NOT auto-removed (prevents breaking `__init__.py` re-exports).
- Every module MUST declare `__all__` (list of exported names) near the top.

## Workflow

After editing Python, run **both** (order matters — `check --fix` applies autofixes, then `format` reflows):

```bash
ruff check --fix <changed_files>
ruff format <changed_files>
```

`--symlink-install` colcon builds mean Python edits take effect on the next node launch **without a rebuild**. C++ changes always require `build` (or `build_python` to skip `*_cpp` packages).

## Module skeleton (canonical)

```python
"""One-line summary.

Longer description of the module's purpose, key classes, and any non-obvious
behavior. Reference related modules with ``:mod:`package.module```.
"""

from __future__ import annotations

import standard_library_module
from standard_library_module import Thing

import third_party_module
from third_party_module import helper

from qtpy.QtCore import Qt, Signal
from qtpy.QtWidgets import QWidget

import rclpy
from rclpy.node import Node

from autoboat_msgs.msg import WaypointList
from ground_station.utils import constants, misc

from .local_module import LocalClass

__all__ = ["PublicClass", "public_function"]


class PublicClass:
    """One-line class summary.

    Parameters
    ----------
    param_name : type
        Description of the parameter.

    Notes
    -----
    Any non-obvious behavior, threading concerns, or lifecycle gotchas.
    """

    def __init__(self, param_name: str) -> None:
        self._field: str = param_name

    def public_method(self, value: float) -> bool:
        """One-line summary of what the method does.

        Parameters
        ----------
        value : float
            Description.

        Returns
        -------
        bool
            Description of the return value.

        Raises
        ------
        ValueError
            When ``value`` is negative.
        """
        if value < 0:
            raise ValueError("value must be non-negative")
        return True
```

## Numpy docstring sections (supported)

`Parameters`, `Returns`, `Yields`, `Raises`, `Notes`, `Examples`, `See Also`, `Inherits`, `References`, `Contains`, `Attributes`. Use `Inherits` when a subclass overrides but keeps the same contract as the parent.

## ROS 2 node patterns

Two autopilot implementations: `ros_packages/autopilot/autopilot/` (Python, `ament_python`) and `ros_packages/autopilot_cpp/` (C++, `ament_cmake`). Python nodes live under `ros_packages/<pkg>/<pkg>/`.

### Node skeleton (representative)

```python
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from autoboat_msgs.msg import WaypointList
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String


class SailboatAutopilotNode(Node):
    def __init__(self) -> None:
        super().__init__("sailboat_autopilot")

        # transient_local QoS for late-joining subscribers (config path, latched state)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # SensorDataQoS for high-rate sensor streams
        sensor_qos = rclpy.qos.sensor_data_qos()

        self.position_subscriber = self.create_subscription(
            NavSatFix, "/position", self._position_callback, sensor_qos
        )
        self.waypoint_publisher = self.create_publisher(WaypointList, "/waypoints", 10)

        # Wall timer drives the control loop
        refresh_rate = 50.0  # Hz — read from params JSON in real code
        self.timer = self.create_timer(1.0 / refresh_rate, self._tick)

    def _position_callback(self, msg: NavSatFix) -> None: ...
    def _tick(self) -> None: ...


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SailboatAutopilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### `TelemetryNode` — the `spin_once` idiom

`ros_packages/autopilot/autopilot/telemetry_node.py` does **not** use `rclpy.spin`. Instead it calls `rclpy.spin_once(node, timeout_sec=0)` inside its own loop so ROS callbacks drain without blocking the HTTP server. Pattern:

```python
while rclpy.ok():
    rclpy.spin_once(self, timeout_sec=0)
    # do non-blocking work (HTTP polling, socket flush, etc.)
    time.sleep(loop_period)
```

Use this only when a node must multiplex ROS with another event source. Prefer `rclpy.spin` for pure ROS nodes.

### Topic name conventions

- All nodes publish/subscribe on `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (set in devcontainer env).
- Canonical topics: `/position` (NavSatFix), `/heading` (Float32), `/desired_rudder_angle`, `/desired_sail_angle`, `/current_rudder_angle`, `/rc_data` (RCData), `/waypoints` (WaypointList), `/autopilot_parameters` (String JSON), `/autopilot_param_config_path` (String, transient_local).
- High-rate sensor streams use `rclpy.qos.sensor_data_qos()` (best-effort, depth 5).
- Latched/config topics use `QoSProfile(depth=1, reliable, transient_local)`.

## Ground Station Qt patterns

The ground station (`ground_station/src/`) is a **PyQt/PySide** desktop app (via `qtpy` abstraction — NEVER import `PyQt5`/`PyQt6`/`PySide2`/`PySide6` directly; always go through `qtpy`). It embeds a `QWebEngineView` running a vanilla-TS Leaflet map widget.

### `StateManager` — persistent config singleton

`ground_station/src/utils/state_manager.py::StateManager` is the singleton (`constants.SM`). Backed by `app_data/git_ignore/app_state.json` (gitignored by name).

**API:**

```python
from ground_station.utils import constants

# write — persists to disk immediately
constants.SM.write("key", value)
constants.SM.write("another", {"nested": [1, 2, 3]})

# typed reads (return None / default if missing or wrong type)
constants.SM.read_int("telemetry_server_instance_id")        # int
constants.SM.read_str("telemetry_server_instance_user")      # str
constants.SM.read_bool("data_logging_active")                # bool
constants.SM.read_float("some_gain")                         # float
constants.SM.read_dict("current_autopilot_parameters")       # dict
```

**Critical rule:** before reading a key, add it to `constants.STATE_FILE_CONTENTS` with a sensible default — `StateManager.__init__` merges missing keys from that dict on startup. Reading a key not in `STATE_FILE_CONTENTS` works but returns `None`/default and is a latent bug (e.g. `keybind_widget` reads `"keybindings"` which is **not** in `STATE_FILE_CONTENTS` — flagged for fix).

### `MapBridge` — the Python→JS bridge (preferred)

**All** map JS calls from Python go through `MapBridge` (`ground_station/src/widgets/map_widget/bridge.py`), a hand-maintained typed wrapper over the TS `MapInterface`. Never call `browser.page().runJavaScript(...)` directly from widget code — add a method to `MapBridge` and call that.

```python
from ground_station.utils import constants
from widgets.map_widget import MapBridge

# in widget __init__, right after the QWebEngineView is created:
self.map_bridge = MapBridge(self.browser)
QTimer.singleShot(0, self.map_bridge.verify_api)   # drift check after load

# call typed methods — args serialized via json.dumps, wrapped in js_load_guard internally:
self.map_bridge.add_waypoint(lat, lon)
self.map_bridge.set_track_visible(True)
self.map_bridge.update_boat_location_and_heading(lat, lon, heading)
self.map_bridge.set_keybinds({"focus_boat": "F"})
```

**Conventions:**
- One `@_map_api`-decorated Python method per TS `MapInterface` method. All fire-and-forget (`-> None`).
- For batched multi-statement JS, call individual `MapBridge` methods in sequence — each is queued independently by `js_load_guard`.
- **Adding a new TS method:** (1) add it to `MapInterface` in `main.ts` (auto-discovered by `getApi()`), (2) add the matching `@_map_api` method to `MapBridge`.
- **Drift detection:** `MapInterface.getApi()` (TS) returns `[{name, params}]` via prototype introspection; `MapBridge.verify_api()` (Python) queries it post-load and logs mismatches via `print` (does not raise).

### `js_load_guard` — low-level JS wrapper (used by `MapBridge`)

`MapBridge._call` wraps every JS string in `misc.js_load_guard(...)`, which polls `typeof map === "object"` until the TS frontend has finished initializing (the `map` global is set by `main.ts` after `new MapInterface(...)`). Without it, calls fire into an empty page and silently no-op. Widget code should NOT call `js_load_guard`/`runJavaScript` directly — use `MapBridge` methods.

> ⚠️ **`js_load_guard` is for fire-and-forget calls only.** It wraps the JS in a function with `return;` (no value) and queues via `__autoboatPendingMapCode` when the map isn't ready. This means the return value of the wrapped JS expression is **discarded** — the Python `runJavaScript` callback receives `undefined`. If you need a return value from JS (e.g. `MapBridge.verify_api()` querying `map.getApi()`), do NOT use `js_load_guard` — call `runJavaScript` directly with a JS expression that returns `null` when not ready, and poll via `QTimer.singleShot` until it returns a value.

> ⚠️ **Docs drift:** an older comment claimed `main.ts` dispatches a `mapLoaded` event that Python listens for. It does **not**. The guard polls `typeof map` on a retry/ready-check. Do not add a `mapLoaded` listener — `MapBridge` handles readiness internally.

### `thread_classes` — the `Router` pattern for QThread fetchers

Background HTTP/ROS work goes through `QThread` subclasses in `ground_station/src/utils/thread_classes.py`. They follow a **Router** pattern: a namespace class holds inner `QThread` subclasses, each emitting a `response` Signal on completion. Callers connect to `response` and start the thread from a `_starter` method gated on `isRunning()`.

```python
from ground_station.utils import thread_classes, misc, constants

# in widget __init__:
self.local_waypoint_handler = thread_classes.WaypointThreadRouter.LocalFetcherThread()
self.local_waypoint_handler.response.connect(self.update_waypoints_display)
self.one_ms_timer.timeout.connect(self.local_waypoint_handler_starter)

def local_waypoint_handler_starter(self) -> None:
    """Timer callback — start the fetcher if it isn't already running."""
    if not self.local_waypoint_handler.isRunning():
        self.local_waypoint_handler.start()
```

**Why the `_starter` indirection?** `QThread.start()` on an already-running thread is a silent no-op; the starter gates on `isRunning()` so a slow HTTP response doesn't get dropped by a faster timer tick.

Other Routers in `thread_classes`: `WaypointThreadRouter` (local + remote waypoint fetch), `InstanceManagerThreadRouter` (telemetry instance list), `ImageFetcher` (camera frames), `BoatStatusFetcherThread` (live boat telemetry, owned by `GraphViewer`).

### Widget registration

To add a new widget:

1. Create `ground_station/src/widgets/<name>/__init__.py` exporting the public class.
2. Add the class name to `ground_station/src/widgets/__init__.py::__all__` and import it there.
3. In `ground_station/src/main.py::MainWindow.load_main_tabs()`, call `self.main_widget.addTab(<WidgetClass>(...), "<Title>")`.

### `GraphViewer` — the telemetry source

`ground_station/src/widgets/graph_viewer.py::GraphViewer` owns `BoatStatusFetcherThread` (polls the telemetry server) and re-emits the boat data tuple on `boat_data_signal = Signal(tuple)`. `GroundStationWidget` and `DataLogger` both connect to this signal — `GroundStationWidget` for display, `DataLogger` for CSV logging.

### Data logging

`DataLogger` (`constants.DL`) writes CSVs to `constants.DATA_LOGS_DIR / f"data_log_{time.time_ns()}.csv"`. Files > 20 MB are gzip-compressed (`.csv.gz`, level 9) on stop. Start/stop toggles the `data_logging_active` StateManager flag and connects/disconnects `constants.DL.write_from_qthread` from `boat_data_signal`.

## Enum sync rule (cross-repo invariant)

`ground_station/src/utils/constants.py::StrictMatchEnums` MUST stay byte-for-byte in sync with `ros_packages/autopilot/autopilot/autopilot_library/utils/constants.py`. Changing one requires changing the other. Known drifts to fix:

- **`MotorboatControlModes`**: GS is missing `EMERGENCY_STOP = 4` (ROS side has it).
- **`TelemetryStatus`**: GS uses `StrEnum` + `auto()` with a `WRONG_FORMAT` member; ROS side uses plain `Enum` without `WRONG_FORMAT`.

`autopilot_cpp` has its own C++ enums in `autopilot_library/autopilot_utils.hpp` (`SailboatControlModes`, `MotorboatControlModes`, `SailboatAutopilotStates`, `SailboatManeuvers`, `PropellerMotorControlMode`) — these are a **third** copy that must also stay aligned.

## Telemetry server HTTP API

The ground station talks to a Flask-style telemetry server (run on the boat's Jetson). Routes are centralized in `constants._*_endpoints` dicts and accessed via `misc.get_route("<name>")` (which fills in the base URL from StateManager). All requests go through `constants.REQ_SESSION` (a `requests.Session`).

Representative endpoints (URL filled by `get_route`):
- `get_default_autopilot_parameters/<id>` — GET defaults JSON
- `get_autopilot_parameters/<id>` — GET current params
- `set_autopilot_parameters/<id>` — POST params dict
- `update_autopilot_parameter` — POST single param update
- `get_all_ids` — GET list of instance IDs
- `get_user_from_id/<id>` — GET username
- `create_instance` — POST new instance
- `get_current_camera_image/<id>` — GET base64 JPEG

## Telemetry instance reconnection logic

`InstanceHandler` (`ground_station/src/widgets/instance_handler.py`) tracks a `connection_history: deque[TelemetryStatus](maxlen=2)`. When a `FAILURE → SUCCESS` transition is detected, it double-checks by re-querying `get_all_ids` + `get_user_from_id` to detect **instance ID reuse** (same ID, different username → treat as a new instance). The `has_telemetry_server_instance_changed` StateManager flag is polled every 10 ms by `MainWindow.check_instance_connection` to trigger `load_main_tabs()`.

## Things to avoid

- Editing files under `build/`, `install/`, `log/`, `__pycache__/`, `*.egg-info/` — colcon build artifacts.
- Importing `PyQt5`/`PyQt6`/`PySide2`/`PySide6` directly — always go through `qtpy`.
- Calling `browser.page().runJavaScript(...)` directly from widget code — go through `MapBridge` (`widgets/map_widget/bridge.py`), which wraps every call in `js_load_guard` and serializes args via `json.dumps`. Add a new `@_map_api` method to `MapBridge` for each new TS `MapInterface` method.
- Reading a StateManager key without first adding it to `constants.STATE_FILE_CONTENTS`.
- Starting a `QThread` directly from a timer callback without the `isRunning()` gate.
- Adding a new ignore to `ruff.toml` without justification in a comment.
- Assuming `rclpy.spin` is the only loop pattern — `TelemetryNode` uses `spin_once` in a manual loop.
- Using `output="screen"` in launch files for high-rate nodes (fills the terminal); prefer `"log"` or commented-out (default).
