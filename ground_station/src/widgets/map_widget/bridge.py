from __future__ import annotations

import inspect
import json
import shutil
import socket
import subprocess
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from qtpy.QtCore import QObject, Qt, QTimer, Signal
from qtpy.QtWebEngineWidgets import QWebEnginePage, QWebEngineView

from utils import constants
from utils.console_logger import get_logger

__all__ = [
    "MapViewerServer",
]

logger = get_logger(__name__)

_LEVEL_LOGGERS: dict[QWebEnginePage.JavaScriptConsoleMessageLevel, Callable[[str], None]] = {
    QWebEnginePage.JavaScriptConsoleMessageLevel.InfoMessageLevel: logger.info,
    QWebEnginePage.JavaScriptConsoleMessageLevel.WarningMessageLevel: logger.warning,
    QWebEnginePage.JavaScriptConsoleMessageLevel.ErrorMessageLevel: logger.error,
}


def _js_load_guard(js_code: str) -> str:
    """Wrap JavaScript so it only runs once the map API exists.

    The map document loads asynchronously, and the frontend module that defines
    :attr:`window.map` executes after the document itself. A call injected before that
    point would throw a `ReferenceError`, so the code is retried on a timer until the
    API is available.

    Polling is used instead of listening for the `mapLoaded` event because that event is
    dispatched exactly once, before any listener registered later could hear it, which
    would silently strand the call forever.
    """

    # note that when using curly braces in f-strings, you need to double them to escape them
    return f"""
        (function () {{
            var attempts = 0;
            var pollIntervalMs = 16;
            var maxAttempts = 600;

            function runWhenMapReady() {{
                if (
                    typeof map !== 'undefined' &&
                    typeof map.update_boat_location_and_heading === 'function'
                ) {{
                    {js_code.strip()}
                    return;
                }}

                attempts += 1;
                if (attempts > maxAttempts) {{
                    console.error('Gave up waiting for the map API before running deferred call.');
                    return;
                }}

                setTimeout(runWhenMapReady, pollIntervalMs);
            }}

            runWhenMapReady();
        }})();
    """.strip()


def _map_api(func: Callable[..., Any]) -> Callable[..., Any]:
    """
    Mark a :class:`MapViewerServer.TypeScriptInterface` method as part of the public API surface.

    Methods decorated with this are included in
    :meth:`MapViewerServer.TypeScriptInterface.get_api_signatures` and compared against the TS side during
    :meth:`MapViewerServer.TypeScriptInterface.verify_api`.
    """

    func._is_map_api = True
    return func


class MapViewerServer(QObject):
    """
    Manage the Vite map viewer server process and the page that renders its frontend.

    :attr:`page` is the :class:`QWebEnginePage` a :class:`QWebEngineView` displays the map in. It is a
    :class:`MapViewerServer.TypeScriptInterface.BrowserLogger`, so the frontend's JavaScript console output
    reaches the ground station logger.

    :class:`MapViewerServer.TypeScriptInterface` is grouped here because it drives the frontend this server
    serves, but the server does not create it - callers instantiate it with the :class:`QWebEngineView` they
    want to drive.

    Parameters
    ----------
    parent
        An optional Qt parent for this :class:`QObject`.

    Attributes
    ----------
    process
        The :class:`subprocess.Popen` object representing the map viewer server process, or `None` if not running.
    page
        The :class:`QWebEnginePage` the map frontend is rendered in, as a
        :class:`MapViewerServer.TypeScriptInterface.BrowserLogger` so its console output is logged.

    Signals
    -------
    server_ready
        Emitted by :meth:`start_and_load` once the viewer is accepting connections. The server connects this
        to :meth:`_load_page` itself, so the page is navigated on the GUI thread even though
        :meth:`start_and_load` is normally called from a worker thread.

    Inherits
    --------
    :class:`QObject`
    """

    server_ready = Signal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)

        self.page = self.TypeScriptInterface.BrowserLogger()
        self._process: subprocess.Popen | None = None
        self._page_loaded = False
        self._log_output_thread: threading.Thread | None = None

        # A queued connection is required, not merely preferred: `start_and_load` runs on a worker thread
        # but `page.load` must touch the page from the GUI thread this object lives on.
        self.server_ready.connect(self._load_page, Qt.ConnectionType.QueuedConnection)

    @property
    def process(self) -> subprocess.Popen | None:
        """The :class:`subprocess.Popen` object representing the map viewer server process, or `None` if not running."""

        return self._process

    @process.setter
    def process(self, value: subprocess.Popen | None) -> None:
        """Set the map viewer server process, or `None` when it is not running."""

        self._process = value

    def _load_page(self) -> None:
        """
        Navigate :attr:`page` to the map viewer URL.

        Invoked through :attr:`server_ready` so it runs on the GUI thread. Navigating before Vite is
        listening makes the first load fail with ``ERR_CONNECTION_REFUSED`` even though a manual reload then
        works, which is why :meth:`start_and_load` only emits after :meth:`wait_until_ready` succeeds.

        Guarded by ``_page_loaded`` so a repeated signal does not reload the page.
        """

        if self._page_loaded:
            return

        self._page_loaded = True
        self.page.load(constants.MAP_URL)
        logger.info(f"Loading map from {constants.MAP_URL.toString()}...")

    def start_and_load(self) -> bool:
        """
        Start the viewer, wait until it accepts connections, then load :attr:`page`.

        This is the single entry point for bringing the map up: it keeps the ordering invariant
        (start -> wait -> navigate) in one place instead of spreading it across a worker thread and a
        polling timer in ``main.py``. Safe to call from a worker thread; the navigation itself is
        marshalled to the GUI thread through :attr:`server_ready`.

        Returns
        -------
        bool
            `True` if the server became ready and the page load was scheduled, `False` otherwise.
        """

        self.start()

        if not self.wait_until_ready():
            return False

        self.server_ready.emit()
        return True

    def wait_until_ready(self, timeout: float = 60.0, poll_interval: float = 0.2) -> bool:
        """
        Block until the map viewer server is accepting TCP connections.

        Parameters
        ----------
        timeout
            Maximum number of seconds to wait before giving up.
        poll_interval
            Seconds to sleep between connection attempts.

        Returns
        -------
        bool
            `True` once the server accepts connections, `False` if it exited or
            the timeout elapsed first.
        """

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if self.process is not None and self.process.poll() is not None:
                logger.error(f"Map viewer server exited with code {self.process.returncode} before becoming ready.")
                return False

            try:
                # attempt to connect to the server to check if it's ready
                with socket.create_connection(("127.0.0.1", constants.MAP_VIEWER_PORT), timeout=1.0):
                    logger.info(f"Map viewer server is accepting connections on port {constants.MAP_VIEWER_PORT}.")
                    return True

            except OSError:
                time.sleep(poll_interval)

        logger.error(f"Map viewer server did not become ready within {timeout:.0f} seconds.")
        return False

    def start(self) -> None:
        """Start the map viewer server process."""

        if self.process is not None and self.process.poll() is None:
            logger.warning("Map viewer server is already running.")
            return

        bun_path = shutil.which("bun")
        if bun_path is None:
            logger.error("Bun is not installed or not found in PATH. Please install Bun to run the map viewer server.")
            return

        bun_path = Path(bun_path).resolve()
        if not bun_path.is_file():
            logger.error(f"Bun executable not found at {bun_path}. Please install Bun to run the map viewer server.")
            return

        try:
            output = subprocess.run(
                [bun_path.as_posix(), "--version"],
                check=True,
                capture_output=True,
                shell=False,
                timeout=10,
            )
            logger.info(f"Using Bun version {output.stdout.decode().strip()}")

        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired, OSError):
            logger.error("Bun is not installed or not found in PATH. Please install Bun to run the map viewer server.")
            return

        logger.info("Making sure all dependencies are installed for the map viewer server...")
        try:
            start_time = time.time()
            subprocess.run(
                [bun_path.as_posix(), "install"],
                check=True,
                cwd=constants.TOP_LEVEL_DIR.as_posix(),
                shell=False,
            )
            elapsed_time = time.time() - start_time
            if elapsed_time < 1.0:
                logger.info("Dependencies were already installed, skipping installation.")
            else:
                logger.info(f"Dependencies installed successfully in {elapsed_time:.2f} seconds.")

        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to install dependencies for the map viewer server: {e}")
            return

        try:
            self.process = subprocess.Popen(
                [bun_path, "run", "serve"],
                cwd=constants.TOP_LEVEL_DIR.as_posix(),
                shell=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self._log_output_thread = threading.Thread(target=self._log_output, daemon=True)
            self._log_output_thread.start()
            logger.info(f"Map viewer server started on port {constants.MAP_VIEWER_PORT}.")

        except Exception as e:
            logger.error(f"Failed to start map viewer server: {e}")
            self.process = None

    def _log_output(self) -> None:
        """
        Forward the map viewer server's output to the ground station logger.

        Vite writes its startup banner and all compile/HMR errors to stdout and stderr. Those streams are piped
        so the lines can be logged to the Qt console instead of only appearing in the terminal that launched the
        application - a failed compile is otherwise invisible from inside the app.
        """

        process = self.process
        if process is None or process.stdout is None:
            return

        for raw_line in process.stdout:
            line = raw_line.strip()

            # `bun run` echoes the script it is about to run as `$ <command>`,
            # which is noise rather than server output.
            if not line or line.startswith("$ "):
                continue

            if "error" in line.lower() or "failed" in line.lower():
                logger.error(f"[vite] {line}")
            elif "warn" in line.lower():
                logger.warning(f"[vite] {line}")
            else:
                logger.info(f"[vite] {line}")

    def stop(self) -> None:
        """Stop the map viewer server process."""

        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            exit_code = self.process.wait()

            if exit_code != 0:
                logger.warning(f"Map viewer server terminated with exit code {exit_code}.")
            else:
                logger.info("Map viewer server stopped successfully.")

        else:
            logger.warning("Map viewer server is not running.")

    class TypeScriptInterface:
        """
        Typed Python wrapper over the Typescript :class:`MapInterface`.

        Parameters
        ----------
        browser
            The :class:`QWebEngineView` the map frontend is rendered in.
        page
            The :class:`QWebEnginePage` hosting the frontend, normally the
            :class:`MapViewerServer.TypeScriptInterface.BrowserLogger` the server provides. Defaults to
            ``browser.page()`` when omitted.
        """

        class BrowserLogger(QWebEnginePage):
            """
            A :class:`QWebEnginePage` that forwards the frontend's JavaScript console output to the ground station logger.

            The map widget is rendered inside a :class:`QWebEngineView`, which has no visible developer console, so
            messages logged from the Typescript frontend (for example the MapTiler key fallback warning in `main.ts`)
            would otherwise never be seen. Every ``console.log``/``console.warn``/``console.error`` is re-emitted as a
            `[map]` log record, which reaches both the Qt console widget and the log file.

            Inherits
            --------
            :class:`QWebEnginePage`
            """

            def javaScriptConsoleMessage(
                self,
                level: QWebEnginePage.JavaScriptConsoleMessageLevel,
                message: str,
                line_number: int,
                source_id: str,
            ) -> None:
                """
                Forward a JavaScript console message to the Qt logger.

                Parameters
                ----------
                level
                    Severity reported by Chromium; mapped to the matching logger method.
                message
                    The text passed to the JavaScript ``console`` call.
                line_number
                    Line number in the source file that logged the message.
                source_id
                    URL of the source file that logged the message.
                """

                log = _LEVEL_LOGGERS.get(level, logger.info)
                source = source_id.rsplit("/", 1)[-1] if source_id else "map"
                log(f"[map] {message} ({source}:{line_number})")

        def __init__(self, browser: QWebEngineView, page: QWebEnginePage | None = None) -> None:
            self._browser = browser
            self._page = page if page is not None else browser.page()
            self._api_verified = False
            self._page_destroyed = False

            # Calls made before the map document has loaded are held here rather than
            # in JavaScript. `runJavaScript` before navigation executes against
            # `about:blank`, and navigating away destroys that window, taking any
            # JS-side queue with it. Python state survives the navigation, so the
            # collect-then-flush-on-loadFinished approach is the only reliable one.
            self._pending_calls: list[str] = []
            self._map_loaded = False

            loaded_page = self._page
            if loaded_page is not None:
                loaded_page.loadStarted.connect(self._on_load_started)
                loaded_page.loadFinished.connect(self._on_load_finished)
                loaded_page.destroyed.connect(self._on_page_destroyed)

                # This interface can be built after the map has already loaded, for
                # example when the ground station tab is created once a telemetry
                # instance is chosen. `loadFinished` will not fire again, so adopt the
                # current state instead of queueing every call forever.
                if self._is_map_document():
                    self._map_loaded = True

        def _call(self, method: str, *args: Any) -> None:
            """
            Build and run a ``map.<method>(args...)`` JS call.

            Parameters
            ----------
            method
                The TS method name on :attr:`window.map`.
            *args
                The arguments to pass to the TS method.  Each is serialized via
                :meth:`json.dumps` to ensure proper quoting and escaping.
            """

            js_args = ", ".join(json.dumps(a) for a in args)
            js = f"map.{method}({js_args})"
            self._run_js(js)

        def _call_raw(self, js: str) -> None:
            """
            Run a pre-built JS expression through the load guard.

            Parameters
            ----------
            js
                The raw JS code to run.
            """

            self._run_js(js)

        def _run_js(self, js: str) -> None:
            """
            Run JS on the map page, deferring it until the map document has loaded.

            Parameters
            ----------
            js
                The JS code to run, executed only once :attr:`window.map` exists.
            """

            if self._page is None or self._page_destroyed:
                return

            if not self._map_loaded:
                self._pending_calls.append(js)
                return

            self._page.runJavaScript(_js_load_guard(js))

        def _flush_pending_calls(self) -> None:
            """
            Run every call that was queued before the map document loaded.

            Order is preserved, which matters for calls that build on each other such
            as restoring several saved map features in sequence.
            """

            if not self._pending_calls:
                return

            pending_calls = self._pending_calls
            self._pending_calls = []

            if self._page is None or self._page_destroyed:
                return

            logger.info(f"Flushing {len(pending_calls)} map call(s) queued before the map loaded.")

            for js in pending_calls:
                self._page.runJavaScript(_js_load_guard(js))

        def get_api_signatures(self) -> dict[str, list[str]]:
            """Return ``{method_name: [param_names]}`` for all ``@_map_api`` methods.

            Returns
            -------
            `dict[str, list[str]]`
                A mapping of method name to the list of parameter names
                (excluding ``self``), mirroring what the TS :meth:`getApi()` returns.
            """

            signatures: dict[str, list[str]] = {}
            for name in dir(self):
                if name.startswith("_"):
                    continue

                attr = getattr(self, name, None)
                if not callable(attr) or not getattr(attr, "_is_map_api", False):
                    continue

                params = [p for p in inspect.signature(attr).parameters if p != "self"]
                signatures[name] = params

            return signatures

        def _on_load_started(self) -> None:
            """Mark the map as not ready while a new document is being navigated to."""

            self._map_loaded = False

        def _on_page_destroyed(self) -> None:
            """
            Record that Qt has destroyed the page's underlying C++ object.

            Calls can still arrive after shutdown starts, because the telemetry fetcher
            emits faster than the close sequence completes and its slots were already
            queued on the GUI thread. Checking this flag makes :meth:`_run_js` a no-op
            instead of raising ``Internal C++ object already deleted``. The dying page is
            left untouched so as not to disturb Qt's destruction sequence.
            """

            self._page_destroyed = True
            self._pending_calls.clear()

        def _is_map_document(self) -> bool:
            """
            Whether the page is currently showing the map viewer document.

            The page starts on `about:blank`, and `loadFinished` fires for that initial
            empty document too. Treating it as ready would let calls run against a window
            that is about to be destroyed by the navigation to :data:`constants.MAP_URL`.

            Returns
            -------
            `bool`
                `True` if the loaded document is the map viewer.
            """

            if self._page is None or self._page_destroyed:
                return False

            page_url = self._page.url().toString()
            return page_url.startswith(constants.MAP_URL.toString())

        def _on_load_finished(self, ok: bool) -> None:
            """
            Flush queued map calls, then run the API drift check.

            Parameters
            ----------
            ok
                Whether the page load succeeded, as reported by :attr:`loadFinished`.
            """

            if not ok:
                logger.warning("Map page failed to load; drift check skipped.")
                return

            if not self._is_map_document():
                return

            self._map_loaded = True
            self._flush_pending_calls()

            self.verify_api()

        def verify_api(self) -> None:
            """
            Compare the TS :class:`MapInterface` methods against the Python ``@_map_api``
            methods and log any mismatches.

            Normally driven by :meth:`_on_load_finished`, but can also be called manually once the page is
            up. Probes :meth:`map.getApi()` a few times to ride out the gap between `loadFinished` and the
            frontend module executing. Bypasses :meth:`_js_load_guard` because we need the return value of
            :meth:`JSON.stringify(map.getApi())` delivered to the Python callback.
            """

            if self._api_verified:
                return

            max_attempts = 50
            delay_ms = 200
            attempts = 0

            # JS that returns the JSON string if the map is ready, or null otherwise.
            js_probe = "(typeof map !== 'undefined' && typeof map.getApi === 'function') ? JSON.stringify(map.getApi()) : null"

            def _on_result(result: Any) -> None:
                nonlocal attempts

                if result is None:
                    attempts += 1
                    if attempts < max_attempts:
                        QTimer.singleShot(delay_ms, _retry)
                    else:
                        logger.warning("map.getApi() never became available; drift check skipped.")

                    return

                self._api_verified = True

                if not isinstance(result, str):
                    logger.warning("map.getApi() did not return a JSON string; drift check skipped.")
                    return

                try:
                    ts_list = json.loads(result)

                except (json.JSONDecodeError, TypeError):
                    logger.warning("map.getApi() returned invalid JSON; drift check skipped.")
                    return

                if not isinstance(ts_list, list):
                    logger.warning("map.getApi() did not return a list; drift check skipped.")
                    return

                ts_sigs: dict[str, int] = {}
                for entry in ts_list:
                    if isinstance(entry, dict) and isinstance(entry.get("name"), str):
                        arity = entry.get("arity")
                        if isinstance(arity, int):
                            ts_sigs[entry["name"]] = arity

                py_sigs: dict[str, int] = {name: len(params) for name, params in self.get_api_signatures().items()}
                ts_names = set(ts_sigs)
                py_names = set(py_sigs)

                missing_in_py = ts_names - py_names
                missing_in_ts = py_names - ts_names

                param_mismatches: list[tuple[str, int, int]] = []
                for name in ts_names & py_names:
                    if ts_sigs[name] != py_sigs[name]:
                        param_mismatches.append((name, ts_sigs[name], py_sigs[name]))

                if not (missing_in_py or missing_in_ts or param_mismatches):
                    logger.info("TypeScriptInterface API matches TS MapInterface.")
                    return

                if missing_in_py:
                    logger.warning(f"TS methods not in TypeScriptInterface: {sorted(missing_in_py)}")

                if missing_in_ts:
                    logger.warning(f"TypeScriptInterface methods not in TS: {sorted(missing_in_ts)}")

                for name, ts_arity, py_arity in param_mismatches:
                    logger.warning(
                        f"Param-arity mismatch for '{name}': TS expects {ts_arity} arg(s), PY expects {py_arity} arg(s)"
                    )

            def _retry() -> None:
                if self._page is None or self._page_destroyed:
                    return

                self._page.runJavaScript(js_probe, _on_result)

            _retry()

        # region exposed MapInterface methods

        @_map_api
        def update_boat_location(self, lat: float, lon: float) -> None:
            """Update the boat marker position without changing heading."""

            self._call("update_boat_location", lat, lon)

        @_map_api
        def update_boat_heading(self, heading: float) -> None:
            """Update the boat marker heading without changing position."""

            self._call("update_boat_heading", heading)

        @_map_api
        def update_boat_location_and_heading(self, lat: float, lon: float, heading: float, record_track: bool = True) -> None:
            """Update the boat marker position and heading in one call."""

            self._call("update_boat_location_and_heading", lat, lon, heading, record_track)

        @_map_api
        def focus_map_on_boat(self) -> None:
            """Center the map on the boat's current position."""

            self._call("focus_map_on_boat")

        @_map_api
        def focus_map_on_marker(self, lat: float, lon: float) -> None:
            """Center the map on a waypoint marker and focus it."""

            self._call("focus_map_on_marker", lat, lon)

        @_map_api
        def focus_map_on_buoy(self, lat: float, lon: float) -> None:
            """Center the map on a buoy marker and focus it."""

            self._call("focus_map_on_buoy", lat, lon)

        @_map_api
        def add_waypoint(self, lat: float, lon: float) -> None:
            """Add a waypoint at the given coordinates."""

            self._call("add_waypoint", lat, lon)

        @_map_api
        def remove_waypoint(self, index: int) -> None:
            """Remove the waypoint at the given index."""

            self._call("remove_waypoint", index)

        @_map_api
        def change_color_waypoints(self, color: str) -> None:
            """Recolor all waypoints with the given color name."""

            self._call("change_color_waypoints", color)

        @_map_api
        def clear_waypoints(self) -> None:
            """Remove all waypoints from the map."""

            self._call("clear_waypoints")

        @_map_api
        def undo_last_waypoint(self) -> None:
            """Undo the last waypoint add or remove operation."""

            self._call("undo_last_waypoint")

        @_map_api
        def add_buoy(self, lat: float, lon: float) -> None:
            """Add a buoy marker at the given coordinates."""

            self._call("add_buoy", lat, lon)

        @_map_api
        def remove_buoy(self, index: int) -> None:
            """Remove the buoy at the given index."""

            self._call("remove_buoy", index)

        @_map_api
        def clear_buoys(self) -> None:
            """Remove all buoys from the map."""

            self._call("clear_buoys")

        @_map_api
        def clear_track(self) -> None:
            """Clear the boat track history."""

            self._call("clear_track")

        @_map_api
        def set_track_visible(self, visible: bool) -> None:
            """Show or hide the boat track layer."""

            self._call("set_track_visible", visible)

        @_map_api
        def set_bathymetry_visible(self, visible: bool) -> None:
            """Show or hide the ocean depth layer."""

            self._call("set_bathymetry_visible", visible)

        @_map_api
        def set_land_boundary_visible(self, visible: bool) -> None:
            """Show or hide the faint ocean/land boundary outline."""

            self._call("set_land_boundary_visible", visible)

        @_map_api
        def toggle_measure(self) -> None:
            """Toggle the distance measure tool on the map."""

            self._call("toggle_measure")

        @_map_api
        def clear_measure(self) -> None:
            """Clear the measured path and distance readout from the map."""

            self._call("clear_measure")

        @_map_api
        def set_latlon_grid_visible(self, visible: bool) -> None:
            """Show or hide the latitude/longitude grid lines."""

            self._call("set_latlon_grid_visible", visible)

        @_map_api
        def remove_all_svgs(self) -> None:
            """Remove all diagnostic SVG overlays from the map."""

            self._call("remove_all_svgs")

        @_map_api
        def update_no_sail_svg(self, inner_html: str, size: float) -> None:
            """Update the no-sail zone SVG overlay."""

            self._call("update_no_sail_svg", inner_html, size)

        @_map_api
        def update_velocity_svg(self, inner_html: str, size: float) -> None:
            """Update the velocity arrow SVG overlay."""

            self._call("update_velocity_svg", inner_html, size)

        @_map_api
        def update_wind_svg(self, inner_html: str) -> None:
            """Update the wind direction SVG overlay."""

            self._call("update_wind_svg", inner_html)

        @_map_api
        def update_compass_svg(self, degree: float) -> None:
            """Update the compass control rotation."""

            self._call("update_compass_svg", degree)

        @_map_api
        def set_keybinds(self, bindings: dict[str, str]) -> None:
            """Push a keybind map ``{action: combo}`` to the TS frontend."""

            self._call("set_keybinds", bindings)

        # endregion exposed MapInterface methods
