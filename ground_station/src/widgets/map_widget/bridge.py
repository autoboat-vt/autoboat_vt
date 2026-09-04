from __future__ import annotations

import inspect
import json
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from qtpy.QtCore import QTimer

from utils.console_logger import get_logger

if TYPE_CHECKING:
    from qtpy.QtWebEngineWidgets import QWebEngineView

__all__ = ["MapBridge"]

logger = get_logger(__name__)


def _js_load_guard(js_code: str) -> str:
    """Run JavaScript after the map API has loaded."""

    # note that when using curly braces in f-strings, you need to double them to escape them
    return f"""
        function __runAutoboatMapCodeWhenReady() {{
            if (
                typeof map !== 'undefined' &&
                typeof map.update_boat_location_and_heading === 'function'
            ) {{
                {js_code.strip()}
                return;
            }}

            window.__autoboatPendingMapCode = __runAutoboatMapCodeWhenReady;

            if (!window.__autoboatPendingMapListenerAdded) {{
                window.__autoboatPendingMapListenerAdded = true;

                document.addEventListener(
                    'mapLoaded',
                    function handleMapLoaded() {{
                        window.__autoboatPendingMapListenerAdded = false;

                        if (window.__autoboatPendingMapCode) {{
                            const pendingMapCode = window.__autoboatPendingMapCode;
                            window.__autoboatPendingMapCode = null;
                            pendingMapCode();
                        }}
                    }},
                    {{ once: true }}
                );
            }}
        }}

        __runAutoboatMapCodeWhenReady();
    """.strip()


def _map_api(func: Callable[..., Any]) -> Callable[..., Any]:
    """
    Mark a :class:`MapBridge` method as part of the public API surface.

    Methods decorated with this are included in :meth:`MapBridge.get_api_signatures`
    and compared against the TS side during :meth:`MapBridge.verify_api`.
    """

    func._is_map_api = True
    return func


class MapBridge:
    """
    Typed Python wrapper over the Typescript :class:`MapInterface`.

    Parameters
    ----------
    browser
        The :class:`QWebEngineView` hosting the map frontend.
    """

    def __init__(self, browser: QWebEngineView) -> None:
        self._browser: QWebEngineView = browser

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
        self._browser.page().runJavaScript(_js_load_guard(js))

    def _call_raw(self, js: str) -> None:
        """
        Run a pre-built JS expression through the load guard.

        Use this for batch calls (multiple :meth:`map.X()` statements joined by
        newlines) where calling individual methods would be impractical.
        """

        self._browser.page().runJavaScript(_js_load_guard(js))

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

    def verify_api(self) -> None:
        """
        Compare the TS :class:`MapInterface` methods against the Python ``@_map_api``
        methods and log any mismatches.

        Polls the frontend until :meth:`map.getApi()` is available (the map global
        may not be set yet when this runs at startup), then compares signatures.
        Bypasses :meth:`js_load_guard` because we need the return value of
        :meth:`JSON.stringify(map.getApi())` delivered to the Python callback.
        """

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

            ts_sigs: dict[str, list[str]] = {}
            for entry in ts_list:
                if isinstance(entry, dict) and isinstance(entry.get("name"), str):
                    ts_sigs[entry["name"]] = list(entry.get("params", []))

            py_sigs = self.get_api_signatures()
            ts_names = set(ts_sigs)
            py_names = set(py_sigs)

            missing_in_py = ts_names - py_names
            missing_in_ts = py_names - ts_names

            def _norm(params: list[str]) -> list[str]:
                """Normalize parameter names for comparison."""

                return [p.replace("_", "").lower() for p in params]

            param_mismatches: list[tuple[str, list[str], list[str]]] = []
            for name in ts_names & py_names:
                if _norm(ts_sigs[name]) != _norm(py_sigs[name]):
                    param_mismatches.append((name, ts_sigs[name], py_sigs[name]))

            if not (missing_in_py or missing_in_ts or param_mismatches):
                logger.info("MapBridge API matches TS MapInterface.")
                return

            if missing_in_py:
                logger.warning(f"TS methods not in MapBridge: {sorted(missing_in_py)}")

            if missing_in_ts:
                logger.warning(f"MapBridge methods not in TS: {sorted(missing_in_ts)}")

            for name, ts_p, py_p in param_mismatches:
                logger.warning(f"Param mismatch for '{name}': TS={ts_p}, PY={py_p}")

        def _retry() -> None:
            self._browser.page().runJavaScript(js_probe, _on_result)

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
    def update_boat_location_and_heading(
        self, lat: float, lon: float, heading: float, record_track: bool = True
    ) -> None:
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
