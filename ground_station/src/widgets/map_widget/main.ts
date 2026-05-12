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
    static readonly buoyColor = "orange";
    static readonly waypointColor = "blue";
    static readonly focusedWaypointColor = "violet";

    lastFocusedTimestamp = performance.now();
    readonly minZoom = 5;
    readonly maxZoom = 20;

    map: LeafletMapType;

    readonly waypoints: WaypointManager;
    readonly buoys: BuoyManager;
    readonly boat: BoatManager;
    readonly svgs: SVGManager;

    static getIcon(color: string): Icon {
        const key = `marker-${color}`;
        const cachedIcon = MapInterface.iconCache.get(key);

        if (cachedIcon !== undefined) {
            return cachedIcon;
        }

        const markerIcon = icon({
            iconUrl: MapInterface.assetsUrl + `marker-icon-${color}.png`,
            shadowUrl: MapInterface.assetsUrl + "marker-shadow.png",
            iconSize: [25, 41],
            iconAnchor: [12, 41],
            shadowSize: [41, 41]
        });

        MapInterface.iconCache.set(key, markerIcon);
        return markerIcon;
    }

    static getBoatIcon(scale = 1): Icon {
        return icon({
            iconUrl: MapInterface.assetsUrl + "boat.png",
            iconSize: [50 * scale, 50 * scale],
            iconAnchor: [25 * scale, 25 * scale]
        });
    }

    constructor() {
        this.map = LeafletMap("map", MapInterface.mapOptions);

        this.waypoints = new WaypointManager(
            this.map,
            MapInterface.getIcon.bind(MapInterface),
            MapInterface.waypointColor,
            MapInterface.focusedWaypointColor,
            this.syncWaypoints.bind(this)
        );

        this.buoys = new BuoyManager(this.map, MapInterface.getIcon.bind(MapInterface), MapInterface.buoyColor);

        this.boat = new BoatManager(this.map, MapInterface.getBoatIcon.bind(MapInterface));

        this.svgs = new SVGManager(this.map, this.boat);

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
            this.waypoints.add(event.latlng.lat, event.latlng.lng);
        });

        this.map.on("contextmenu", (event: LeafletMouseEvent) => {
            const closestIndex = this.waypoints.findClosestIndex(event.latlng.lat, event.latlng.lng, 20);

            if (closestIndex !== -1) {
                this.waypoints.remove(closestIndex);
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
        if (!this.waypoints.focusedWaypoint) {
            return;
        }

        const diff = performance.now() - this.lastFocusedTimestamp;
        if (diff > 500) {
            this.waypoints.unfocus();
        }
    }

    update_boat_location(lat: number, lon: number): void {
        this.boat.updateLocation(lat, lon);
    }

    update_boat_heading(heading: number): void {
        this.boat.updateHeading(heading);
    }

    update_boat_location_and_heading(lat: number, lon: number, heading: number): void {
        this.boat.updateLocationAndHeading(lat, lon, heading);
    }

    focus_map_on_boat(): void {
        this.boat.focus();
    }

    focus_map_on_marker(lat: number, lon: number): void {
        this.map.setView([lat, lon], this.map.getMaxZoom());
        this.waypoints.focus(lat, lon);
        this.lastFocusedTimestamp = performance.now();
    }

    focus_map_on_buoy(lat: number, lon: number): void {
        this.map.setView([lat, lon], this.map.getMaxZoom());
        this.waypoints.focus(lat, lon);
    }

    add_waypoint(lat: number, lon: number): void {
        this.waypoints.add(lat, lon);
    }

    remove_waypoint(index: number): void {
        this.waypoints.remove(index);
    }

    change_color_waypoints(color: string): void {
        this.waypoints.changeColor(color);
    }

    clear_waypoints(): void {
        this.waypoints.clear();
    }

    add_buoy(lat: number, lon: number): void {
        this.buoys.add(lat, lon);
    }

    remove_buoy(index: number): void {
        this.buoys.remove(index);
    }

    clear_buoys(): void {
        this.buoys.clear();
    }

    remove_all_svgs(): void {
        this.svgs.removeAllSvgs();
    }

    update_no_sail_svg(innerHTML: string, size: number): void {
        this.svgs.updateNoSailSvg(innerHTML, size);
    }

    update_velocity_svg(innerHTML: string, size: number): void {
        this.svgs.updateVelocitySvg(innerHTML, size);
    }

    update_wind_svg(innerHTML: string): void {
        this.svgs.updateWindSvg(innerHTML);
    }

    update_compass_svg(degree: number): void {
        this.svgs.updateCompassSvg(degree);
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
