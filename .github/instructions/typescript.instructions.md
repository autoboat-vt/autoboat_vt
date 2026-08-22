---
description: "Use when writing or editing TypeScript/JavaScript/CSS/HTML in the ground_station map widget frontend. Covers Biome config, the Vite-served Leaflet map, MapInterface class, MarkerManager abstract base, manager pattern, keybinds.ts, SVGManager gotchas, Python→JS bridge, and tsconfig hard-scoping rules."
applyTo: "**/*.ts, **/*.js, **/*.css, **/*.html, **/*.jsonc, **/biome.json, **/biome.jsonc"
---

# TypeScript / Frontend Conventions

All TS/JS/CSS/HTML/JSON is handled by **Biome** (root `biome.jsonc`). Config summary:

- 4-space indent, 130-char line width (HTML/CSS 100).
- Double quotes, semicolons always, trailing commas none, arrow parens always.
- `linter.preset = "recommended"` with: `useConst: error`, `useTemplate: warn`, `noConsole: off`, `noDebugger: error`, `noUnusedImports: error`, `noUnusedVariables: warn`.
- `files.includes` excludes `__pycache__`, `node_modules`, `build`, `dist`, etc. Don't add TS files under those paths.

## Workflow

```bash
cd ground_station
biome check --write .
```

If Biome's file count looks wrong (e.g. files outside `ground_station/` being linted, or `public/` not excluded), a parent directory's `biome.json` is being picked up. Run with explicit config: `biome check --config-path=./biome.json .` to verify. (See user memory `biome_config.md` for the full gotcha.)

## Map Widget Frontend (ground_station/src/widgets/map_widget/frontend/)

This is the **only** TS surface in the repo. It is a vanilla-TS Leaflet app — **no React, Vue, Svelte, router, or state management library**. Deps are `leaflet` + `leaflet-rotatedmarker` only.

### Hard tsconfig scoping

- `tsconfig.json` `include` is scoped to `src/widgets/map_widget/frontend/*` (`.ts`, `.js`, `.d.ts`).
- New TS files for the map widget MUST live in that directory. Files placed elsewhere are invisible to the TS compiler.
- Strict mode is on: `noUncheckedIndexedAccess`, `noImplicitOverride`, `noUnusedLocals`, `noUnusedParameters`, `noFallthroughCasesInSwitch`.

### Vite

- `vite.config.ts` root = `src/widgets/map_widget/frontend`, publicDir = `app_data/`.
- Dev server: `127.0.0.1:{VITE_PORT}` (default `5173`), `strictPort: true`, `hmr: false`.
- Started by `ground_station/run.sh` via `bun run serve` alongside the PyQt app (`src/main.py`).
- **Ports come from `ground_station/server_ports.env`** (single source of truth shared with `run.sh` and `constants.py`). Because Vite's `loadEnv` only reads files named `.env*`, `vite.config.ts` uses a small `parseEnvFile()` helper (via `node:fs.readFileSync`) to read `server_ports.env` directly, then uses `define` to inject the values into client code as `import.meta.env.MAP_SERVER_PORT` / `import.meta.env.ASSET_SERVER_PORT`. Do not hardcode port numbers in TS — read them from `import.meta.env` (typed in `frontend/global.d.ts` via `/// <reference types="vite/client" />` + `interface ImportMetaEnv` augmentation) with `??` fallback defaults.

## Python↔JS bridge

- `main.ts` exports a singleton `MapInterface` instance attached to `window.map` (`window.map = new MapInterface(...)`).
- **Python calls JS through `MapBridge`** (`ground_station/src/widgets/map_widget/bridge.py`), a hand-maintained class with one typed method per public `MapInterface` method. Never call `browser.page().runJavaScript(...)` directly from widget code — add a method to `MapBridge` and call that instead. `MapBridge` internally wraps every call in `misc.js_load_guard(...)` and serializes args via `json.dumps`, so callers just pass Python values.
- `MapBridge` is constructed with the `QWebEngineView` (`self.map_bridge = MapBridge(self.browser)`) and exposes methods like `add_waypoint(lat, lon)`, `clear_waypoints()`, `set_track_visible(visible)`, `set_keybinds(bindings_dict)`, `update_boat_location_and_heading(lat, lon, heading)`, etc. Adding a new TS method requires adding the matching Python method to `MapBridge` (decorated with `@_map_api`).
- For batched multi-statement JS (e.g. several SVG updates at once), call the individual `MapBridge` methods in sequence rather than building a single JS string — each is queued by `js_load_guard` independently.
- **Drift detection:** `MapInterface.getApi()` (TS) returns `[{name, params}]` for every public method via prototype introspection. `MapBridge.verify_api()` (Python) queries it after map load and logs mismatches (TS methods missing from Python, Python methods missing from TS, param count/reorder mismatches) via `print`. It does not raise — the bridge still works; this is informational. `verify_api` is called automatically on startup (`QTimer.singleShot(0, self.map_bridge.verify_api)` in `GroundStationWidget.__init__`).
  - **Excluded TS methods:** `getApi` (introspection), `handleMapMove` (internal event handler), and `syncWaypoints` (TS→Python callback, not a Python→JS call) are excluded from `getApi()` output via an `exclude` Set. If you add a new TS method that Python should NOT call, add it to that Set.
  - **Param name normalization:** Python uses snake_case (`inner_html`), TS uses camelCase (`innerHTML`). Since `MapBridge` passes args positionally, `verify_api` normalizes param names (strip `_`, lowercase) before comparing — only a count mismatch or genuine reordering triggers a warning, not a naming-style difference.

> ⚠️ **Docs drift to avoid:** an older comment in the codebase claimed `main.ts` dispatches a `mapLoaded` CustomEvent that Python listens for. It does **not**. There is no event — `js_load_guard` polls `typeof map` (via a retry/ready-check wrapper). Do not add a `mapLoaded` listener.

- Frontend exposes methods on `MapInterface` (e.g. `focus_map_on_boat`, `add_waypoint`, `clear_waypoints`, `change_color_waypoints`, `set_keybinds`, `getApi`). Keep the public API stable — Python calls it by string name.
- TS→Python side of the bridge: `WaypointManager` takes a `syncWaypoints: (waypoints: LatLngTuple[]) => void` callback (bound to `MapInterface.syncWaypoints`) and calls it on every mutation via the `afterChange` hook. It POSTs the waypoint list to `MapInterface.waypointsUrl` (`http://localhost:{MAP_SERVER_PORT}/waypoints`, where the port comes from `import.meta.env.MAP_SERVER_PORT` injected by `vite.config.ts` from `ground_station/server_ports.env`) — the Python `ThreadingHTTPServer` started by `map_widget/server.py`.

## `MapInterface` class

The singleton `MapInterface` is the public surface Python touches. Constructor wires up Leaflet + all managers:

```ts
export class MapInterface {
    private map: LeafletMap;
    private boatManager: BoatManager;
    private buoyManager: BuoyManager;
    private waypointManager: WaypointManager;
    private svgManager: SVGManager;
    private keybindHandler: KeybindHandler;

    constructor() {
        this.map = map("map", { center: [37.23, -80.42], zoom: 15, /* ... */ });
        this.boatManager = new BoatManager(this.map);
        this.buoyManager = new BuoyManager(this.map);
        this.waypointManager = new WaypointManager(this.map, this.syncWaypoints.bind(this));
        this.svgManager = new SVGManager(this.map);
        this.keybindHandler = new KeybindHandler(this.map, /* keybinds from Python */);
    }

    // public methods called from Python via runJavaScript:
    add_waypoint(lat: number, lon: number): void { this.waypointManager.add(lat, lon); }
    clear_waypoints(): void { this.waypointManager.clear(); }
    change_color_waypoints(color: string): void { this.waypointManager.changeColor(color); }
    set_keybinds(bindings: KeybindMap): void { this.keybindHandler.updateBindings(bindings); }
    // ... etc

    private syncWaypoints(waypoints: LatLngTuple[]): void {
        // POST to MapInterface.waypointsUrl (http://localhost:{MAP_SERVER_PORT}/waypoints,
        // where MAP_SERVER_PORT comes from import.meta.env, injected by vite.config.ts
        // from ground_station/server_ports.env)
        fetch(MapInterface.waypointsUrl, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ waypoints }),
        });
    }
}
```

## Manager pattern

`MapInterface` owns composable managers: `BoatManager`, `BuoyManager`, `WaypointManager`, `SVGManager`, `KeybindHandler`, `TrackManager`. Add new map features as a new manager class following the same pattern. `iconCache: Map<string, Icon>` is static — reuse icons, don't recreate per marker.

## `TrackManager`

File: `ground_station/src/widgets/map_widget/frontend/track.ts`. Records a history of recent boat positions rendered as **individual point markers** (Leaflet `CircleMarker`s, not a connected polyline).

- `TrackManager.record(lat, lon)` is called from `MapInterface.update_boat_location` and `update_boat_location_and_heading` on every boat position update.
- Each recorded point is a `circleMarker` with fixed pixel radius (`POINT_RADIUS_PX = 3`), so point size is independent of zoom.
- Points are pruned FIFO at `DEFAULT_MAX_POINTS = 1000` (~17 min at 1 Hz telemetry); the oldest `CircleMarker` is removed from the map when pruned.
- GPS jitter filter: drops points within `MIN_DISTANCE_METERS = 0.5` of the last recorded point.
- No discontinuity handling (no polyline to draw a long line across the map); each point stands alone.
- `setVisible(bool)` toggles layer visibility without losing accumulated history. `clear()` wipes history.
- Track color is teal (`#0891b2`) to stay distinct from blue waypoints, orange buoys, and violet focused waypoints.
- **Toggle lives in Map Appearance Configuration**, not on the map or in the waypoint tab. `MapInterface.set_track_visible(visible: boolean)` is the JS bridge method. The ground station's `MapOptionsHandler` dialog (opened via the "Map Appearance Configuration" button) has a "Boat Track" row driven by `constants._map_features["boat_track"]`; toggling it fires `GroundStationWidget.on_map_feature_toggled("boat_track", enabled)` which calls `map.set_track_visible(...)` via `runJavaScript(misc.js_load_guard(...))`. On startup, if `boat_track` status is persisted as enabled, the ground station calls `map.set_track_visible(true)` to restore the layer.
- `MapInterface.clear_track()` exposes history clearing to Python if needed (not currently called from Python).

## `MarkerManager` (abstract base)

File: `ground_station/src/widgets/map_widget/frontend/marker.ts`. `BoatManager`, `BuoyManager`, `WaypointManager` all extend this. Three parallel data structures keyed by `"lat,lon"` string:

```ts
export abstract class MarkerManager {
    readonly points: LatLngTuple[] = [];                 // ordered array
    readonly markers = new Map<string, Marker>();        // Leaflet Marker lookup
    readonly markerColors = new Map<string, string>();   // stored color

    constructor(
        protected readonly map: LeafletMap,
        protected readonly getIcon: (color: string) => Icon
    ) {}

    findClosestIndex(lat: number, lon: number): number { /* uses Leaflet distanceTo */ }

    protected makeKey(lat: number, lon: number): string { return `${lat},${lon}`; }

    protected addPoint(lat: number, lon: number, color: string): void {
        const key = this.makeKey(lat, lon);
        if (this.markers.has(key)) { return; }   // dedupe by lat,lon
        // ... push to points, create marker, set in maps, call afterChange()
    }

    protected removePoint(index: number): void { /* ... */ }
    protected popPoint(index: number): LatLngTuple | null { /* ... */ }
    protected clearPoints(): void { /* ... */ }
    protected recolorPoints(color: string): void { /* ... */ }

    // Hooks subclasses override:
    protected getDisplayColor(_key: string, storedColor: string): string { return storedColor; }
    protected afterPointRemoved(_key: string): void {}
    protected afterClear(): void {}
    protected afterChange(): void {}
}
```

**Key gotcha — `makeKey` dedupes by lat,lon:** two waypoints at the same `(lat, lon)` will NOT be added twice. If Python pushes two waypoints that round to the same coords, the second is silently dropped. Don't rely on insertion order alone for duplicate detection.

**`findClosestIndex`** uses Leaflet's `distanceTo` (meters); used by `contextmenu` (right-click) to find the nearest waypoint to remove.

## `WaypointManager` (representative subclass)

File: `ground_station/src/widgets/map_widget/frontend/waypoint.ts`.

- Constructor takes a `syncWaypoints` callback (bound to `MapInterface.syncWaypoints`) and calls it on every mutation via the `afterChange` hook.
- Two colors: `defaultColor = "blue"`, `focusedColor = "violet"`.
- `focus(lat, lon)` / `unfocus()` swap the icon to violet; `unfocus` restores the stored color.
- Hooks overridden: `getDisplayColor`, `afterPointRemoved`, `afterClear`, `afterChange`.
- `waypoints = this.points` — read-only view alias of the base class `points` array.

```ts
export class WaypointManager extends MarkerManager {
    private static readonly defaultColor = "blue";
    private static readonly focusedColor = "violet";
    private focusedWaypoint: string | null = null;

    constructor(map: LeafletMap, private readonly syncWaypoints: (w: LatLngTuple[]) => void) {
        super(map, WaypointManager.getIcon);
    }

    get waypoints(): LatLngTuple[] { return this.points; }

    add(lat: number, lon: number): void { this.addPoint(lat, lon, WaypointManager.defaultColor); }
    remove(index: number): void { this.removePoint(index); }
    clear(): void { this.clearPoints(); }
    changeColor(color: string): void { this.recolorPoints(color); }

    focus(lat: number, lon: number): void { /* swap to focusedColor, restore previous */ }
    unfocus(): void { /* restore stored color */ }

    protected override getDisplayColor(key: string, storedColor: string): string {
        return this.focusedWaypoint === key ? WaypointManager.focusedColor : storedColor;
    }
    protected override afterChange(): void { void this.syncWaypoints(this.waypoints); }
}
```

## `SVGManager` + `CompassControl`

File: `ground_station/src/widgets/map_widget/frontend/svg.ts`. Manages **4 SVG-backed things**:

| Field | Type | Position | viewBox |
|-------|------|----------|---------|
| `no_sail_svg` | `SVGOverlay` | relative to boat | `0 0 4 4` |
| `velocity_svg` | `SVGOverlay` | relative to boat | `0 0 4 4` |
| `wind_svg` | `SVGOverlay` | lower-left region | `0 0 100 100` |
| `compass_control` | `CompassControl` (a Leaflet `Control`, NOT an `SVGOverlay`) | fixed bottom-left viewport | inline SVG |

- The wind arrow sits inside `#wind-arrow` and is rotated by `setWindDirection(degrees)` via SVG `transform="rotate(degrees 50 50)"` (negated, around center 50,50).
- `CompassControl` is a `Control` subclass — it lives in the bottom-left of the map viewport and does NOT move with the map.

> ⚠️ **Known bug in `removeAllSvgs()`:** it sets a local `svg = null` inside `forEach` but does NOT clear the `this.no_sail_svg`/`this.wind_svg`/`this.velocity_svg`/`this.compass_control` fields, and `svg_list` was captured at construction (all nulls). So `removeAllSvgs()` is effectively a no-op on first call, and after it does remove overlays, subsequent `update_*` calls take the "update bounds + innerHTML" path on a now-removed overlay. If you touch this code, fix it to null the fields and rebuild `svg_list` dynamically.

## `keybinds.ts` — `KeybindHandler` + `KeybindMap`

File: `ground_station/src/widgets/map_widget/frontend/keybinds.ts`.

```ts
export type KeybindMap = Record<string, string>;   // action_name -> combo string ("Ctrl+Shift+P")
export type KeybindCallback = () => void;

const MODIFIER_KEYS = new Set([
    "Control", "Shift", "Alt", "Meta",
    "ControlLeft", "ControlRight", "ShiftLeft", "ShiftRight",
    "AltLeft", "AltRight", "MetaLeft", "MetaRight"
]);

// Normalize a KeyboardEvent into the form used by the Python KeybindManager.
// Returns an empty string for bare modifier presses (so pressing Shift alone
// does not clear or trigger anything).
function keyboardEventToCombo(event: KeyboardEvent): string {
    if (MODIFIER_KEYS.has(event.key)) { return ""; }
    const parts: string[] = [];
    if (event.ctrlKey) { parts.push("Ctrl"); }
    if (event.shiftKey) { parts.push("Shift"); }
    if (event.altKey) { parts.push("Alt"); }
    if (event.metaKey) { parts.push("Meta"); }
    let keyName = event.key;
    if (keyName.length === 1) { keyName = keyName.toUpperCase(); }   // single chars uppercased
    parts.push(keyName);
    return parts.join("+");
}
```

**Normalization rules** (must match the Python `KeybindManager`):
- Modifiers in fixed order: `Ctrl`, `Shift`, `Alt`, `Meta`.
- Single-character keys uppercased (`a` → `A`).
- Bare modifier presses return `""` (ignored).
- Combo string is `modifier+modifier+...+key` joined with `+`.

The `KeybindHandler` registers a single `keydown` listener on the map container and dispatches to callbacks by looking up `keyboardEventToCombo(event)` in the `KeybindMap`. Bindings are pushed from Python via `map.set_keybinds({...})`.

## Things to avoid

- Adding new TS files outside `ground_station/src/widgets/map_widget/frontend/` — `tsconfig.json` `include` is hard-scoped to that directory.
- Recreating icons per marker — use the static `iconCache: Map<string, Icon>`.
- Calling `browser.page().runJavaScript(...)` from Python without `misc.js_load_guard(...)`.
- Relying on a `mapLoaded` event — it doesn't exist; the guard polls `typeof map`.
- Adding React/Vue/Svelte/router — the frontend is intentionally vanilla TS + Leaflet only.
- Pushing two waypoints at the same `(lat, lon)` — `MarkerManager.makeKey` dedupes silently.
- Importing `PyQt5`/`PyQt6` from any TS-adjacent Python — always `qtpy`.
