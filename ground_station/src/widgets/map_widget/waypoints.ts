import {
    marker,
    type Icon,
    type Map as LeafletMap,
    type Marker
} from "leaflet";
import type { LatLngTuple } from "./types";

export class WaypointManager {
    readonly waypoints: LatLngTuple[] = [];
    readonly markers = new Map<string, Marker>();
    focusedWaypoint: string | null = null;
    lastFocusedColor: string;

    constructor(
        private readonly map: LeafletMap,
        private readonly getIcon: (color: string) => Icon,
        private readonly defaultColor: string,
        private readonly focusedColor: string,
        private readonly syncWaypoints: (
            waypoints: LatLngTuple[]
        ) => Promise<void>
    ) {
        this.lastFocusedColor = defaultColor;
    }

    private makeKey(lat: number, lon: number): string {
        return `${lat.toFixed(6)},${lon.toFixed(6)}`;
    }

    /**
     * Adds a waypoint unless one already exists at the same coordinates.
     */
    add(lat: number, lon: number): void {
        const key = this.makeKey(lat, lon);

        if (this.markers.has(key)) {
            return;
        }

        this.waypoints.push([lat, lon]);

        const waypointMarker = marker([lat, lon], {
            icon: this.getIcon(this.defaultColor)
        }).addTo(this.map);

        this.markers.set(key, waypointMarker);
        void this.syncWaypoints(this.waypoints);
    }

    /**
     * Removes the waypoint at the given index.
     */
    remove(index: number): void {
        if (index < 0 || index >= this.waypoints.length) {
            return;
        }

        const waypoint = this.waypoints.splice(index, 1)[0];
        if (!waypoint) {
            return;
        }

        const [lat, lon] = waypoint;
        const key = this.makeKey(lat, lon);
        const waypointMarker = this.markers.get(key);

        if (waypointMarker) {
            this.map.removeLayer(waypointMarker);
            this.markers.delete(key);
        }

        if (this.focusedWaypoint === key) {
            this.focusedWaypoint = null;
        }

        void this.syncWaypoints(this.waypoints);
    }

    /**
     * Removes all waypoints.
     */
    clear(): void {
        this.markers.forEach(waypointMarker => {
            this.map.removeLayer(waypointMarker);
        });

        this.markers.clear();
        this.waypoints.length = 0;
        this.focusedWaypoint = null;

        void this.syncWaypoints(this.waypoints);
    }

    /**
     * Changes the icon color of all waypoints.
     */
    changeColor(color: string): void {
        this.markers.forEach(waypointMarker => {
            waypointMarker.setIcon(this.getIcon(color));
        });
    }

    /**
     * Clears the focused waypoint state.
     */
    unfocus(): void {
        if (!this.focusedWaypoint) {
            return;
        }

        const waypointMarker = this.markers.get(this.focusedWaypoint);
        if (waypointMarker) {
            waypointMarker.setIcon(this.getIcon(this.lastFocusedColor));
        }

        this.focusedWaypoint = null;
    }

    /**
     * Focuses the waypoint at the given coordinates.
     */
    focus(lat: number, lon: number): void {
        const key = this.makeKey(lat, lon);
        const waypointMarker = this.markers.get(key);

        if (!waypointMarker) {
            return;
        }

        if (this.focusedWaypoint && this.focusedWaypoint !== key) {
            const previousMarker = this.markers.get(this.focusedWaypoint);
            if (previousMarker) {
                previousMarker.setIcon(this.getIcon(this.lastFocusedColor));
            }
        }

        const targetIcon = waypointMarker.options.icon as Icon | undefined;
        const iconUrl = targetIcon?.options.iconUrl;

        this.lastFocusedColor = iconUrl
            ? (iconUrl.split("-").slice(-1)[0]?.replace(".png", "") ??
              this.defaultColor)
            : this.defaultColor;

        this.focusedWaypoint = key;
        waypointMarker.setIcon(this.getIcon(this.focusedColor));
    }

    /**
     * Returns the index of the closest waypoint within the given distance.
     */
    findClosestIndex(lat: number, lon: number, maxDistance = 0.05): number {
        let closestIndex = -1;
        let closestDistance = Number.POSITIVE_INFINITY;

        this.waypoints.forEach((waypoint, index) => {
            const distance = Math.sqrt(
                (waypoint[0] - lat) ** 2 + (waypoint[1] - lon) ** 2
            );

            if (distance < closestDistance && distance < maxDistance) {
                closestIndex = index;
                closestDistance = distance;
            }
        });

        return closestIndex;
    }

    /**
     * Returns the waypoint marker at the given coordinates.
     */
    getMarker(lat: number, lon: number): Marker | undefined {
        return this.markers.get(this.makeKey(lat, lon));
    }
}
