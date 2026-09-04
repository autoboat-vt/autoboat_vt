import {
    control,
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

import { BathymetryManager } from "./bathymetry";
import { BoatManager } from "./boat";
import { BuoyManager } from "./buoys";
import { KeybindHandler, type KeybindMap } from "./keybinds";
import { SVGManager } from "./svg";
import { TrackManager } from "./track";
import type { LatLngTuple } from "./types";
import { WaypointManager } from "./waypoints";

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
        maxBoundsViscosity: 1.0
    };
    static readonly iconCache = new Map<string, Icon>();
    static readonly assetsUrl = `http://localhost:${import.meta.env.ASSET_SERVER_PORT ?? "8000"}`;
    static readonly waypointsUrl = `http://localhost:${import.meta.env.MAP_SERVER_PORT ?? "3002"}/waypoints`;
    static readonly checkLandUrl = `http://localhost:${import.meta.env.MAP_SERVER_PORT ?? "3002"}/check_land`;
    static readonly bathymetryUrl = `http://localhost:${import.meta.env.MAP_SERVER_PORT ?? "3002"}/bathymetry`;

    lastFocusedTimestamp = 0;
    private waypointHistory: { type: "add" | "remove"; waypoint: LatLngTuple; color?: string }[] = [];

    map: LeafletMapType;

    readonly waypoint_manager: WaypointManager;
    readonly buoy_manager: BuoyManager;
    readonly boat_manager: BoatManager;
    readonly svg_manager: SVGManager;
    readonly keybind_handler: KeybindHandler;
    readonly track_manager: TrackManager;
    readonly bathymetry_manager: BathymetryManager;

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
            iconUrl: new URL("boat-icon.png", MapInterface.assetsUrl).toString(),
            iconSize: [50 * scale, 50 * scale],
            iconAnchor: [25 * scale, 25 * scale]
        });
    }

    constructor() {
        this.map = LeafletMap("map", MapInterface.mapOptions);
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

        const mapTilerKey = "M9yBkV9J49pYUg5o8SGC";
        tileLayer(`https://api.maptiler.com/maps/openstreetmap/{z}/{x}/{y}.jpg?key=${mapTilerKey}`, {
            minZoom: MapInterface.MIN_ZOOM,
            maxZoom: MapInterface.MAX_ZOOM,
            tileSize: 512,
            zoomOffset: -1,
            noWrap: true,
            attribution:
                '<a href="https://www.maptiler.com/copyright/" target="_blank">&copy; MapTiler</a> <a href="https://www.openstreetmap.org/copyright" target="_blank">&copy; OpenStreetMap contributors</a>',
            crossOrigin: true
        }).addTo(this.map);

        control.scale().addTo(this.map);

        let moveTimeout: ReturnType<typeof setTimeout> | undefined;
        this.map.on("move", () => {
            if (moveTimeout !== undefined) {
                clearTimeout(moveTimeout);
            }

            moveTimeout = setTimeout(() => this.handleMapMove(), 100);
        });

        this.map.on("click", (event: LeafletMouseEvent) => {
            void this.handleMapClick(event.latlng.lat, event.latlng.lng);
        });

        // contextmenu is right click
        this.map.on("contextmenu", (event: LeafletMouseEvent) => {
            const closestIndex = this.waypoint_manager.findClosestIndex(event.latlng.lat, event.latlng.lng);

            if (closestIndex !== -1) {
                const waypoint = this.waypoint_manager.waypoints[closestIndex];
                if (waypoint) {
                    const color = this.waypoint_manager.getColor(waypoint[0], waypoint[1]);
                    const removed = this.waypoint_manager.pop(closestIndex);
                    if (removed) {
                        this.waypointHistory.push({ type: "remove", waypoint: removed, color });
                    }
                }
            }
        });
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
        if (!(await this.shouldAddWaypoint(lat, lon))) {
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
                console.error("Failed to check land status");
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
                console.error("Failed to sync waypoints");
            }
        } catch (error) {
            console.error("Error syncing waypoints:", error);
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
        }
    }

    change_color_waypoints(color: string): void {
        this.waypoint_manager.changeColor(color);
    }

    clear_waypoints(): void {
        this.waypoint_manager.clear();
        this.waypointHistory = [];
    }

    undo_last_waypoint(): void {
        const op = this.waypointHistory.pop();
        if (!op) {
            return;
        }

        if (op.type === "add") {
            const index = this.waypoint_manager.findClosestIndex(op.waypoint[0], op.waypoint[1]);
            if (index !== -1) {
                this.waypoint_manager.pop(index);
            }
        } else {
            this.waypoint_manager.add(op.waypoint[0], op.waypoint[1], op.color);
        }
    }

    add_buoy(lat: number, lon: number): void {
        this.buoy_manager.add(lat, lon);
    }

    remove_buoy(index: number): void {
        this.buoy_manager.remove(index);
    }

    clear_buoys(): void {
        this.buoy_manager.clear();
    }

    clear_track(): void {
        this.track_manager.clear();
    }

    set_track_visible(visible: boolean): void {
        this.track_manager.setVisible(visible);
    }

    set_bathymetry_visible(visible: boolean): void {
        void this.bathymetry_manager.setVisible(visible);
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
     * capturing the method name and the names of its parameters. Used by the
     * Python MapBridge to detect drift between the two sides at runtime.
     *
     * Returns
     * -------
     * Array<{name: string, params: string[]}>
     */
    getApi(): Array<{ name: string; params: string[] }> {
        const api: Array<{ name: string; params: string[] }> = [];

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
            // strip leading/trailing whitespace and parens from the param list
            const raw = String(fn).slice(0, String(fn).indexOf(")"));
            const paramStart = raw.indexOf("(");
            const paramList = paramStart === -1 ? "" : raw.slice(paramStart + 1);
            const params = paramList
                .split(",")
                .map((p) => p.trim())
                .filter((p) => p.length > 0 && p !== "this");
            api.push({ name, params });
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

export default map;
