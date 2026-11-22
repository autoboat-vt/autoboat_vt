import {
    type Icon,
    icon,
    map as LeafletMap,
    type Map as LeafletMapType,
    type LeafletMouseEvent,
    type MapOptions,
    tileLayer
} from "leaflet";
import "leaflet/dist/leaflet.css";
import "leaflet-rotatedmarker";
import "./styles/lat_lon_grid.css";
import "./styles/measure.css";
import "./styles/zoom_scale_row.css";

import { BathymetryManager } from "./bathymetry";
import { BoatManager } from "./boat";
import { BuoyManager } from "./buoys";
import { KeybindHandler, type KeybindMap } from "./keybinds";
import { LandBoundaryManager } from "./land_boundary";
import { LatLonGridManager } from "./lat_lon_grid";
import { MeasureManager } from "./measure";
import { SVGManager } from "./svg";
import { TrackManager } from "./track";
import type { LatLngTuple } from "./types";
import { WaypointManager } from "./waypoints";
import { ZoomScaleRowControl } from "./zoom_scale_row";

class MapInterface {
    // note that lower zoom levels are more zoomed out
    // and higher zoom levels are more zoomed in
    static readonly MIN_ZOOM = 3;
    static readonly MAX_ZOOM = 20;
    static readonly mapOptions: MapOptions = {
        center: [0, 0],
        zoom: 13,
        minZoom: MapInterface.MIN_ZOOM,
        maxZoom: MapInterface.MAX_ZOOM,
        preferCanvas: true,
        maxBounds: [
            [-90, -180],
            [90, 180]
        ],
        maxBoundsViscosity: 1.0,
        attributionControl: true,

        // the zoom buttons are added manually below, so they can be positioned
        // in the bottom right corner above the attribution line
        zoomControl: false
    };

    static readonly iconCache = new Map<string, Icon>();
    static readonly assetsUrl = `http://localhost:${import.meta.env.ASSET_SERVER_PORT ?? "8000"}`;
    static readonly waypointsUrl = `http://localhost:${import.meta.env.MAP_CALLBACK_PORT ?? "8001"}/waypoints`;
    static readonly checkLandUrl = `http://localhost:${import.meta.env.MAP_CALLBACK_PORT ?? "8001"}/check_land`;
    static readonly bathymetryUrl = `http://localhost:${import.meta.env.MAP_CALLBACK_PORT ?? "8001"}/bathymetry`;
    static readonly landBoundaryUrl = `http://localhost:${import.meta.env.MAP_CALLBACK_PORT ?? "8001"}/land_boundary`;

    // MapTiler keys, tried in order. A revoked or over-quota key still answers
    // with a valid (error) tile image instead of failing the request, so the map
    // would silently go blank; instead the next key is used.
    static readonly mapTilerKeys = ["21bF5koC68eFHXyf2U6o", "cgMC3WGf9i7Di4lU79UT"];

    lastFocusedTimestamp = 0;
    private waypointHistory: { type: "add" | "remove"; waypoint: LatLngTuple; color?: string }[] = [];

    /**
     * Waypoint adds awaiting a log line. `add_waypoint` is called once per
     * waypoint when Python restores a mission or runs the 500-waypoint test,
     * so adds are coalesced and reported as a single summary instead of
     * flooding the Qt console with one line per waypoint.
     */
    private pendingWaypointAdds: LatLngTuple[] = [];
    private waypointAddLogTimeout: ReturnType<typeof setTimeout> | undefined;

    map: LeafletMapType;

    readonly waypoint_manager: WaypointManager;
    readonly buoy_manager: BuoyManager;
    readonly boat_manager: BoatManager;
    readonly svg_manager: SVGManager;
    readonly keybind_handler: KeybindHandler;
    readonly track_manager: TrackManager;
    readonly bathymetry_manager: BathymetryManager;
    readonly land_boundary_manager: LandBoundaryManager;
    readonly measure_manager: MeasureManager;
    readonly lat_lon_grid_manager: LatLonGridManager;

    static getMarkerIcon(color: string): Icon {
        const key = `marker-${color}`;
        const cachedIcon = MapInterface.iconCache.get(key);

        if (cachedIcon !== undefined) {
            return cachedIcon;
        }

        const markerIcon = icon({
            iconUrl: new URL(`marker-icon-${color}.png`, MapInterface.assetsUrl).toString(),
            shadowUrl: new URL("marker-shadow.png", MapInterface.assetsUrl).toString(),
            iconSize: [25, 41],
            iconAnchor: [12, 41],
            shadowSize: [41, 41]
        });

        MapInterface.iconCache.set(key, markerIcon);
        return markerIcon;
    }

    static getBoatIcon(scale = 1): Icon {
        return icon({
            iconUrl: new URL("boat-icon.svg", MapInterface.assetsUrl).toString(),
            iconSize: [50 * scale, 50 * scale],
            iconAnchor: [25 * scale, 25 * scale]
        });
    }

    /**
     * Add the MapTiler base layer, using the first key that still works.
     *
     * A revoked or over-quota key makes MapTiler answer `403` with a valid PNG
     * saying the key is invalid, so Leaflet's `tileerror` never fires and the
     * map would just go blank. The keys are therefore probed with `fetch`,
     * which can read the status code, before a layer is created.
     */
    async _add_map_tiler_layer(): Promise<void> {
        const index = await this._find_working_map_tiler_key();
        if (index === -1) {
            console.error("All MapTiler keys are failing, the base map will stay blank");
            return;
        }

        const key = MapInterface.mapTilerKeys[index];
        console.info(`Using MapTiler key ${index + 1} of ${MapInterface.mapTilerKeys.length}`);

        const layer = tileLayer(`https://api.maptiler.com/maps/openstreetmap/{z}/{x}/{y}.jpg?key=${key}`, {
            minZoom: MapInterface.MIN_ZOOM,
            maxZoom: MapInterface.MAX_ZOOM,
            tileSize: 512,
            zoomOffset: -1,
            noWrap: true,
            attribution:
                '<a href="https://www.maptiler.com/copyright/" target="_blank">&copy; MapTiler</a> <a href="https://www.openstreetmap.org/copyright" target="_blank">&copy; OpenStreetMap contributors</a>',
            crossOrigin: true
        }).addTo(this.map);

        // Tiles fail in bulk when the network drops, so warn only once per layer
        // instead of on every failed tile, which would flood the log while panning.
        let tileErrorLogged = false;
        layer.on("tileerror", () => {
            if (tileErrorLogged) {
                return;
            }

            tileErrorLogged = true;
            console.warn("Some map tiles failed to load, the base map may be incomplete");
        });

        // have to set max bounds after adding the maptiler layer, otherwise it will be overridden
        // and the map will be able to pan outside of the bounds
        this.map.setMaxBounds(MapInterface.mapOptions.maxBounds);
    }

    /**
     * Probe each MapTiler key with a single tile request.
     *
     * Returns the index of the first key that returns a real tile. When the
     * network itself is unreachable the first key is returned so the layer is
     * still created and its tiles load once connectivity returns; `-1` means
     * every key answered and rejected the request.
     */
    async _find_working_map_tiler_key(): Promise<number> {
        for (let index = 0; index < MapInterface.mapTilerKeys.length; index += 1) {
            const key = MapInterface.mapTilerKeys[index];
            try {
                const response = await fetch(`https://api.maptiler.com/maps/openstreetmap/0/0/0.jpg?key=${key}`, {
                    method: "HEAD"
                });
                if (response.ok) {
                    return index;
                }
                console.warn(`MapTiler key ${index + 1} is not usable (${response.status}), trying the next key`);
            } catch (error) {
                console.error("Could not reach MapTiler, using the first key:", error);
                return 0;
            }
        }

        return -1;
    }

    constructor() {
        console.info("Initializing map interface");
        this.map = LeafletMap("map", MapInterface.mapOptions);
        void this._add_map_tiler_layer();

        this.waypoint_manager = new WaypointManager(
            this.map,
            MapInterface.getMarkerIcon.bind(MapInterface),
            this.syncWaypoints.bind(this)
        );
        this.buoy_manager = new BuoyManager(this.map, MapInterface.getMarkerIcon.bind(MapInterface));
        this.boat_manager = new BoatManager(this.map, MapInterface.getBoatIcon.bind(MapInterface));
        this.svg_manager = new SVGManager(this.map);
        this.keybind_handler = new KeybindHandler();
        this.track_manager = new TrackManager(this.map);
        this.bathymetry_manager = new BathymetryManager(this.map, MapInterface.bathymetryUrl);
        this.land_boundary_manager = new LandBoundaryManager(this.map, MapInterface.landBoundaryUrl);

        // Every control lives in the bottom right corner, so the left edge stays
        // free for the latitude labels. The scale bar and zoom buttons are added
        // together by ZoomScaleRowControl so they sit side by side (scale on the
        // left) instead of stacking, and the measure tool is registered last so
        // it renders on top of them.
        new ZoomScaleRowControl().addTo(this.map);

        this.measure_manager = new MeasureManager(this.map);
        this.lat_lon_grid_manager = new LatLonGridManager(this.map);

        this.map.createPane("bathyPane");
        const bathyPane = this.map.getPane("bathyPane");
        if (bathyPane !== undefined) {
            bathyPane.style.zIndex = "250";
        }

        this.keybind_handler.register("focus_boat", () => this.focus_map_on_boat());
        this.keybind_handler.register("clear_waypoints", () => this.clear_waypoints());
        this.keybind_handler.register("zoom_in", () => this.map.zoomIn());
        this.keybind_handler.register("zoom_out", () => this.map.zoomOut());
        this.keybind_handler.register("undo_waypoint", () => this.undo_last_waypoint());
        this.keybind_handler.register("toggle_measure", () => this.measure_manager.toggle());

        let moveTimeout: ReturnType<typeof setTimeout> | undefined;
        this.map.on("move", () => {
            if (moveTimeout !== undefined) {
                clearTimeout(moveTimeout);
            }

            moveTimeout = setTimeout(() => this.handleMapMove(), 100);
        });

        this.map.on("click", (event: LeafletMouseEvent) => {
            if (this.measure_manager.isMeasuring()) {
                return;
            }

            void this.handleMapClick(event.latlng.lat, event.latlng.lng);
        });

        // contextmenu is right click
        this.map.on("contextmenu", (event: LeafletMouseEvent) => {
            if (this.measure_manager.isMeasuring()) {
                return;
            }

            const closestIndex = this.waypoint_manager.findClosestIndex(event.latlng.lat, event.latlng.lng);

            if (closestIndex !== -1) {
                const waypoint = this.waypoint_manager.waypoints[closestIndex];
                if (waypoint) {
                    const color = this.waypoint_manager.getColor(waypoint[0], waypoint[1]);
                    const removed = this.waypoint_manager.pop(closestIndex);
                    if (removed) {
                        this.waypointHistory.push({ type: "remove", waypoint: removed, color });
                        console.info(`Removed waypoint at ${removed[0].toFixed(5)}, ${removed[1].toFixed(5)}`);
                    }
                }
            }
        });

        console.info("Map interface ready");
    }

    /**
     * Handle a map click: add a waypoint after confirming with the user if the
     * click is on land.
     *
     * The click coordinates are always checked against the backend's Natural
     * Earth ocean layer first. When the backend reports the point is on land,
     * it blocks on a Qt confirmation dialog; the response tells us whether the
     * user chose to add the waypoint anyway.
     */
    async handleMapClick(lat: number, lon: number): Promise<void> {
        console.info(`Map click at ${lat.toFixed(5)}, ${lon.toFixed(5)}`);

        if (!(await this.shouldAddWaypoint(lat, lon))) {
            console.info("Waypoint placement declined");
            return;
        }

        this.add_waypoint(lat, lon);
    }

    /**
     * Ask the backend whether a waypoint at the given coordinates should be added.
     *
     * The backend checks the point against its land layer. If the point is on
     * water, the backend answers immediately. If the point is on land, the backend
     * blocks on a Qt confirmation dialog and this method waits for the user's
     * answer. Returns `true` on any error so waypoint placement is never blocked
     * by a backend or network problem.
     */
    async shouldAddWaypoint(lat: number, lon: number): Promise<boolean> {
        const url = `${MapInterface.checkLandUrl}?lat=${lat}&lon=${lon}`;
        try {
            const response = await fetch(url);
            if (!response.ok) {
                console.error(`Land check failed (HTTP ${response.status}), adding the waypoint anyway`);
                return true;
            }

            const data = (await response.json()) as { on_land?: boolean; add_waypoint?: boolean };
            return data.add_waypoint === true;
        } catch (error) {
            console.error("Error checking land status:", error);
            return true;
        }
    }

    /**
     * Syncs the current waypoint list with the backend.
     */
    async syncWaypoints(waypoints: LatLngTuple[]): Promise<void> {
        const snapshot = [...waypoints];
        try {
            const response = await fetch(MapInterface.waypointsUrl, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ waypoints: snapshot })
            });

            if (!response.ok) {
                console.error(`Failed to sync ${snapshot.length} waypoints (HTTP ${response.status})`);
            }
        } catch (error) {
            console.error(`Error syncing ${snapshot.length} waypoints:`, error);
        }
    }

    /**
     * Clears waypoint focus after the map has moved for long enough.
     */
    handleMapMove(): void {
        if (!this.waypoint_manager.focusedWaypoint) {
            return;
        }

        const diff = performance.now() - this.lastFocusedTimestamp;
        if (diff > 500) {
            this.waypoint_manager.unfocus();
        }
    }

    update_boat_location(lat: number, lon: number): void {
        this.boat_manager.setLocation(lat, lon);
        this.track_manager.record(lat, lon);
    }

    update_boat_heading(heading: number): void {
        this.boat_manager.setHeading(heading);
    }

    update_boat_location_and_heading(lat: number, lon: number, heading: number, recordTrack: boolean): void {
        this.boat_manager.setLocationAndHeading(lat, lon, heading);
        if (recordTrack) {
            this.track_manager.record(lat, lon);
        } else {
            this.track_manager.clear();
        }
    }

    focus_map_on_boat(): void {
        this.boat_manager.focus();
    }

    focus_map_on_marker(lat: number, lon: number): void {
        this.map.setView([lat, lon], this.map.getMaxZoom());
        this.waypoint_manager.focus(lat, lon);
        this.lastFocusedTimestamp = performance.now();
    }

    focus_map_on_buoy(lat: number, lon: number): void {
        this.map.setView([lat, lon], this.map.getMaxZoom());
        this.waypoint_manager.focus(lat, lon);
    }

    add_waypoint(lat: number, lon: number): void {
        this.waypoint_manager.add(lat, lon);
        this.waypointHistory.push({ type: "add", waypoint: [lat, lon] });
        this._queue_waypoint_add_log(lat, lon);
    }

    /**
     * Report waypoint adds as one log line per burst.
     *
     * A single clicked waypoint logs immediately on the next tick; a bulk add
     * (mission restore, test waypoints) collapses into one summary line.
     */
    private _queue_waypoint_add_log(lat: number, lon: number): void {
        this.pendingWaypointAdds.push([lat, lon]);

        if (this.waypointAddLogTimeout !== undefined) {
            return;
        }

        this.waypointAddLogTimeout = setTimeout(() => {
            this.waypointAddLogTimeout = undefined;
            const added = this.pendingWaypointAdds;
            this.pendingWaypointAdds = [];

            if (added.length === 1) {
                const [only] = added;
                if (only) {
                    console.info(
                        `Added waypoint at ${only[0].toFixed(5)}, ${only[1].toFixed(5)} (${this.waypoint_manager.waypoints.length} total)`
                    );
                }
                return;
            }

            console.info(`Added ${added.length} waypoints (${this.waypoint_manager.waypoints.length} total on the map)`);
        }, 0);
    }

    remove_waypoint(index: number): void {
        const waypoint = this.waypoint_manager.waypoints[index];
        if (!waypoint) {
            return;
        }
        const color = this.waypoint_manager.getColor(waypoint[0], waypoint[1]);
        const removed = this.waypoint_manager.pop(index);
        if (removed) {
            this.waypointHistory.push({ type: "remove", waypoint: removed, color });
            console.info(
                `Removed waypoint at ${removed[0].toFixed(5)}, ${removed[1].toFixed(5)} (${this.waypoint_manager.waypoints.length} left)`
            );
        }
    }

    change_color_waypoints(color: string): void {
        this.waypoint_manager.changeColor(color);
        console.info(`Changed all waypoints to ${color}`);
    }

    clear_waypoints(): void {
        const count = this.waypoint_manager.waypoints.length;
        this.waypoint_manager.clear();
        this.waypointHistory = [];
        console.info(`Cleared ${count} waypoints`);
    }

    undo_last_waypoint(): void {
        const op = this.waypointHistory.pop();
        if (!op) {
            console.info("Nothing to undo");
            return;
        }

        if (op.type === "add") {
            const index = this.waypoint_manager.findClosestIndex(op.waypoint[0], op.waypoint[1]);
            if (index !== -1) {
                this.waypoint_manager.pop(index);
            }
            console.info(`Undid adding a waypoint at ${op.waypoint[0].toFixed(5)}, ${op.waypoint[1].toFixed(5)}`);
        } else {
            this.waypoint_manager.add(op.waypoint[0], op.waypoint[1], op.color);
            console.info(`Undid removing a waypoint at ${op.waypoint[0].toFixed(5)}, ${op.waypoint[1].toFixed(5)}`);
        }
    }

    add_buoy(lat: number, lon: number): void {
        this.buoy_manager.add(lat, lon);
        console.info(`Added buoy at ${lat.toFixed(5)}, ${lon.toFixed(5)}`);
    }

    remove_buoy(index: number): void {
        this.buoy_manager.remove(index);
        console.info(`Removed buoy ${index}`);
    }

    clear_buoys(): void {
        this.buoy_manager.clear();
        console.info("Cleared all buoys");
    }

    clear_track(): void {
        this.track_manager.clear();
        console.info("Cleared the boat track");
    }

    set_track_visible(visible: boolean): void {
        this.track_manager.setVisible(visible);
        console.info(`Boat track ${visible ? "shown" : "hidden"}`);
    }

    set_bathymetry_visible(visible: boolean): void {
        console.info(`Bathymetry layer ${visible ? "enabled" : "disabled"}`);
        void this.bathymetry_manager.setVisible(visible);
    }

    set_land_boundary_visible(visible: boolean): void {
        console.info(`Land boundary layer ${visible ? "enabled" : "disabled"}`);
        void this.land_boundary_manager.setVisible(visible);
    }

    toggle_measure(): void {
        this.measure_manager.toggle();
        console.info(`Measure tool ${this.measure_manager.isMeasuring() ? "started" : "finished"}`);
    }

    clear_measure(): void {
        this.measure_manager.clear();
        console.info("Cleared the measurement");
    }

    set_latlon_grid_visible(visible: boolean): void {
        this.lat_lon_grid_manager.setVisible(visible);
        console.info(`Lat/lon grid ${visible ? "shown" : "hidden"}`);
    }

    remove_all_svgs(): void {
        this.svg_manager.removeAllSvgs();
    }

    update_no_sail_svg(innerHTML: string, size: number): void {
        this.svg_manager.updateNoSailSvg(innerHTML, size, this.boat_manager.getLocation());
    }

    update_velocity_svg(innerHTML: string, size: number): void {
        this.svg_manager.updateVelocitySvg(innerHTML, size, this.boat_manager.getLocation());
    }

    update_wind_svg(innerHTML: string): void {
        this.svg_manager.updateWindSvg(innerHTML);
    }

    update_compass_svg(degree: number): void {
        this.svg_manager.updateCompassSvg(degree);
    }

    set_keybinds(bindings: KeybindMap): void {
        this.keybind_handler.setBindings(bindings);
    }

    /**
     * Introspect the public API of MapInterface.
     *
     * Returns one entry per public method (those not starting with "_"),
     * capturing the method name and its *arity* (number of declared
     * parameters). Used by the Python MapBridge to detect drift between the
     * two sides at runtime.
     *
     * Parameter *names* are not reported because Vite/Terser minifies them to
     * single letters in the production bundle (e.g. `lat` -> `e`). `fn.length`
     * on the other hand is a property of the function object itself and
     * survives minification.
     *
     * Returns
     * -------
     * Array<{name: string, arity: number}>
     */
    getApi(): Array<{ name: string; arity: number }> {
        const api: Array<{ name: string; arity: number }> = [];

        // Methods that are internal to the TS side (introspection, event
        // handlers, TS->Python callbacks) and are not part of the Python->JS
        // API surface. Excluding them here keeps MapBridge.verify_api quiet.
        const exclude = new Set(["getApi", "handleMapClick", "handleMapMove", "shouldAddWaypoint", "syncWaypoints"]);

        const proto = Object.getPrototypeOf(this) as object;
        for (const name of Object.getOwnPropertyNames(proto)) {
            if (name.startsWith("_") || name === "constructor" || exclude.has(name)) {
                continue;
            }
            const descriptor = Object.getOwnPropertyDescriptor(proto, name);
            if (descriptor === undefined || typeof descriptor.value !== "function") {
                continue;
            }
            const fn = descriptor.value as (...args: unknown[]) => unknown;
            // fn.length = number of declared parameters before the first
            // one with a default value / rest arg. It is preserved across
            // minification (the *names* are mangled but the count is not).
            api.push({ name, arity: fn.length });
        }
        return api;
    }
}

const map = new MapInterface();

declare global {
    interface Window {
        map: MapInterface;
    }
}

window.map = map;

// Lets the Python side know the map API is ready. MapBridge's load guard waits
// for this event, so a call made before the module finished loading runs as
// soon as the map exists instead of being dropped.
document.dispatchEvent(new Event("mapLoaded"));

export default map;
