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

import { BoatManager } from "./boat";
import { BuoyManager } from "./buoys";
import { KeybindHandler, type KeybindMap } from "./keybinds";
import { SVGManager } from "./svg";
import { TrackManager } from "./track";
import type { LatLngTuple } from "./types";
import { WaypointManager } from "./waypoints";

class MapInterface {
    static readonly mapOptions: MapOptions = {
        center: [0, 0],
        zoom: 13,
        preferCanvas: true
    };
    static readonly iconCache = new Map<string, Icon>();
    static readonly assetsUrl = "http://localhost:8000";

    // note that lower zoom levels are more zoomed out
    // and higher zoom levels are more zoomed in
    readonly minZoom = 3;
    readonly maxZoom = 20;

    lastFocusedTimestamp = 0;
    private waypointHistory: { type: "add" | "remove"; waypoint: LatLngTuple; color?: string }[] = [];

    map: LeafletMapType;

    readonly waypoint_manager: WaypointManager;
    readonly buoy_manager: BuoyManager;
    readonly boat_manager: BoatManager;
    readonly svg_manager: SVGManager;
    readonly keybind_handler: KeybindHandler;
    readonly track_manager: TrackManager;

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

        this.keybind_handler.register("focus_boat", () => this.focus_map_on_boat());
        this.keybind_handler.register("clear_waypoints", () => this.clear_waypoints());
        this.keybind_handler.register("zoom_in", () => this.map.zoomIn());
        this.keybind_handler.register("zoom_out", () => this.map.zoomOut());
        this.keybind_handler.register("undo_waypoint", () => this.undo_last_waypoint());

        const mapTilerKey = "M9yBkV9J49pYUg5o8SGC";
        tileLayer(`https://api.maptiler.com/maps/openstreetmap/{z}/{x}/{y}.jpg?key=${mapTilerKey}`, {
            minZoom: this.minZoom,
            maxZoom: this.maxZoom,
            tileSize: 512,
            zoomOffset: -1,
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
            this.waypoint_manager.add(event.latlng.lat, event.latlng.lng);
            this.waypointHistory.push({ type: "add", waypoint: [event.latlng.lat, event.latlng.lng] });
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
     * Syncs the current waypoint list with the backend.
     */
    async syncWaypoints(waypoints: LatLngTuple[]): Promise<void> {
        const snapshot = [...waypoints];
        try {
            const response = await fetch("http://localhost:3002/waypoints", {
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

    update_boat_location_and_heading(lat: number, lon: number, heading: number): void {
        this.boat_manager.setLocationAndHeading(lat, lon, heading);
        this.track_manager.record(lat, lon);
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
}

const map = new MapInterface();

declare global {
    interface Window {
        map: MapInterface;
    }
}

window.map = map;

export default map;
