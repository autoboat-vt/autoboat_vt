import { type LatLngExpression, type Map as LeafletMap, type Polyline, polyline } from "leaflet";

import type { LatLngTuple } from "./types";

/** Stroke used for the planned path. */
const PATH_STYLE = {
    color: "#d946ef",
    weight: 4,
    opacity: 0.9,
    dashArray: "8 6"
};

/**
 * Read-only Leaflet polyline overlay showing the path the autopilot actually plans.
 *
 * The path is owned by the boat's autopilot (it is planned there and published on
 * ``/waypoint_path``), so this layer is purely a display: it is never editable and
 * its points are replaced wholesale on every update. It renders into the shared low
 * pane and is non-interactive, so it never intercepts waypoint clicks.
 */
export class PlannedPathManager {
    private layer: Polyline | null = null;
    private visible = true;

    constructor(private readonly map: LeafletMap) {}

    /**
     * Replaces the displayed path.
     *
     * @param points - The path as an ordered list of ``[lat, lon]`` points.
     */
    setPath(points: LatLngTuple[]): void {
        if (this.layer !== null) {
            this.map.removeLayer(this.layer);
            this.layer = null;
        }

        if (points.length < 2) {
            return;
        }

        this.layer = polyline(points satisfies LatLngExpression[], {
            ...PATH_STYLE,
            pane: "bathyPane",
            interactive: false
        });

        if (this.visible) {
            this.layer.addTo(this.map);
        }
    }

    /**
     * Shows or hides the planned path.
     *
     * @param visible - Whether the path should be shown.
     */
    setVisible(visible: boolean): void {
        this.visible = visible;

        if (this.layer === null) {
            return;
        }

        if (visible) {
            this.layer.addTo(this.map);
        } else {
            this.map.removeLayer(this.layer);
        }
    }

    /** Removes the displayed path entirely. */
    clear(): void {
        this.setPath([]);
    }
}
