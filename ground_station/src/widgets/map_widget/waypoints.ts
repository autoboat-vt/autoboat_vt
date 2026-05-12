import type { Icon, Map as LeafletMap } from "leaflet";
import { MarkerManager } from "./marker";
import type { LatLngTuple } from "./types";

export class WaypointManager extends MarkerManager {
    readonly waypoints = this.points;
    focusedWaypoint: string | null = null;

    constructor(
        map: LeafletMap,
        getIcon: (color: string) => Icon,
        private readonly defaultColor: string,
        private readonly focusedColor: string,
        private readonly syncWaypoints: (waypoints: LatLngTuple[]) => Promise<void>
    ) {
        super(map, getIcon);
    }

    add(lat: number, lon: number): void {
        this.addPoint(lat, lon, this.defaultColor);
    }

    remove(index: number): void {
        this.removePoint(index);
    }

    clear(): void {
        this.clearPoints();
    }

    changeColor(color: string): void {
        this.recolorPoints(color);
    }

    unfocus(): void {
        if (!this.focusedWaypoint) {
            return;
        }
        const key = this.focusedWaypoint;
        const waypointMarker = this.markers.get(key);
        const storedColor = this.markerColors.get(key) ?? this.defaultColor;

        if (waypointMarker) {
            waypointMarker.setIcon(this.getIcon(storedColor));
        }

        this.focusedWaypoint = null;
    }

    focus(lat: number, lon: number): void {
        const key = this.makeKey(lat, lon);
        const waypointMarker = this.markers.get(key);

        if (!waypointMarker) {
            return;
        }

        if (this.focusedWaypoint && this.focusedWaypoint !== key) {
            const previousKey = this.focusedWaypoint;
            const previousMarker = this.markers.get(previousKey);
            const previousColor = this.markerColors.get(previousKey) ?? this.defaultColor;
            if (previousMarker) {
                previousMarker.setIcon(this.getIcon(previousColor));
            }
        }

        this.focusedWaypoint = key;
        waypointMarker.setIcon(this.getIcon(this.focusedColor));
    }

    protected override getDisplayColor(key: string, storedColor: string): string {
        if (this.focusedWaypoint === key) {
            return this.focusedColor;
        }
        return storedColor;
    }

    protected override afterPointRemoved(key: string): void {
        if (this.focusedWaypoint === key) {
            this.focusedWaypoint = null;
        }
    }

    protected override afterClear(): void {
        this.focusedWaypoint = null;
    }

    protected override afterChange(): void {
        void this.syncWaypoints(this.waypoints);
    }
}
