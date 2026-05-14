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
import { SVGManager } from "./svg";
import type { LatLngTuple } from "./types";
import { WaypointManager } from "./waypoints";

class MapInterface {
    static readonly mapOptions: MapOptions = { center: [0, 0], zoom: 13 };
    static readonly iconCache = new Map<string, Icon>();
    static readonly assetsUrl = "http://localhost:8000/";

    lastFocusedTimestamp = performance.now();
    readonly minZoom = 5;
    readonly maxZoom = 20;

    map: LeafletMapType;

    readonly waypoint_manager: WaypointManager;
    readonly buoy_manager: BuoyManager;
    readonly boat_manager: BoatManager;
    readonly svg_manager: SVGManager;

    static getMarkerIcon(color: string): Icon {
        const key = `marker-${color}`;
        const cachedIcon = MapInterface.iconCache.get(key);

        if (cachedIcon !== undefined) {
            return cachedIcon;
        }

        const markerIcon = icon({
            iconUrl: `${MapInterface.assetsUrl}marker-icon-${color}.png`,
            shadowUrl: `${MapInterface.assetsUrl}marker-shadow.png`,
            iconSize: [25, 41],
            iconAnchor: [12, 41],
            shadowSize: [41, 41]
        });

        MapInterface.iconCache.set(key, markerIcon);
        return markerIcon;
    }

    static getBoatIcon(scale = 1): Icon {
        return icon({
            iconUrl: `${MapInterface.assetsUrl}boat.png`,
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
        });

        // contextmenu is right click
        this.map.on("contextmenu", (event: LeafletMouseEvent) => {
            const closestIndex = this.waypoint_manager.findClosestIndex(event.latlng.lat, event.latlng.lng);

            if (closestIndex !== -1) {
                this.waypoint_manager.remove(closestIndex);
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
    }

    update_boat_heading(heading: number): void {
        this.boat_manager.setHeading(heading);
    }

    update_boat_location_and_heading(lat: number, lon: number, heading: number): void {
        this.boat_manager.setLocationAndHeading(lat, lon, heading);
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
        this.waypoint_manager.remove(index);
    }

    change_color_waypoints(color: string): void {
        this.waypoint_manager.changeColor(color);
    }

    clear_waypoints(): void {
        this.waypoint_manager.clear();
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
}

const map = new MapInterface();

declare global {
    interface Window {
        map: MapInterface;
    }
}

window.map = map;

export default map;
