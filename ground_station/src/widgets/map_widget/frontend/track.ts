import { type CircleMarker, circleMarker, type Map as LeafletMap } from "leaflet";
import type { LatLngTuple } from "./types";

/**
 * Maximum number of points to retain. Older points are pruned (FIFO) as new
 * ones arrive. At 1 Hz telemetry this is ~17 minutes of history.
 */
const DEFAULT_MAX_POINTS = 1000;

/**
 * Minimum distance in meters between two consecutive recorded points. Points
 * closer than this to the last recorded point are dropped to avoid GPS jitter
 * producing a noisy cluster.
 */
const MIN_DISTANCE_METERS = 0.5;

/**
 * Color of the track points.
 */
const TRACK_COLOR = "#0891b2";

/**
 * Radius in pixels of each track point marker. Fixed size regardless of zoom.
 */
const POINT_RADIUS_PX = 3;

/**
 * Manages a history of recent boat positions rendered as individual point
 * markers (Leaflet `CircleMarker`s) rather than a connected polyline.
 *
 * Points are recorded via {@link record}. The layer can be toggled visible or
 * hidden without losing accumulated history.
 */
export class TrackManager {
    private readonly maxPoints: number;
    private readonly points: CircleMarker[] = [];
    private visible = false;
    private lastPoint: LatLngTuple | null = null;

    constructor(
        private readonly map: LeafletMap,
        maxPoints = DEFAULT_MAX_POINTS
    ) {
        this.maxPoints = maxPoints;
    }

    /**
     * Record a boat position.
     *
     * Drops points within {@link MIN_DISTANCE_METERS} of the last recorded
     * point (GPS jitter filter).
     *
     * @param lat - Latitude in decimal degrees.
     * @param lon - Longitude in decimal degrees.
     */
    record(lat: number, lon: number): void {
        const candidate: LatLngTuple = [lat, lon];

        if (this.lastPoint !== null) {
            const distance = this.haversine(this.lastPoint, candidate);
            if (distance < MIN_DISTANCE_METERS) {
                return;
            }
        }

        const marker = circleMarker(candidate, {
            radius: POINT_RADIUS_PX,
            color: TRACK_COLOR,
            fillColor: TRACK_COLOR,
            fillOpacity: 0.8,
            weight: 1
        });
        if (this.visible) {
            marker.addTo(this.map);
        }
        this.points.push(marker);
        this.lastPoint = candidate;

        if (this.points.length > this.maxPoints) {
            const oldest = this.points.shift();
            if (oldest !== undefined) {
                this.map.removeLayer(oldest);
            }
        }
    }

    /**
     * Toggle layer visibility without losing accumulated history.
     */
    setVisible(visible: boolean): void {
        if (this.visible === visible) {
            return;
        }
        this.visible = visible;
        for (const point of this.points) {
            if (visible) {
                point.addTo(this.map);
            } else {
                this.map.removeLayer(point);
            }
        }
    }

    /**
     * Whether the track layer is currently visible on the map.
     */
    isVisible(): boolean {
        return this.visible;
    }

    /**
     * Clear all recorded history and remove the layer from the map.
     */
    clear(): void {
        for (const point of this.points) {
            this.map.removeLayer(point);
        }
        this.points.length = 0;
        this.lastPoint = null;
    }

    /**
     * Haversine great-circle distance in meters between two lat/lon points.
     */
    private haversine(a: LatLngTuple, b: LatLngTuple): number {
        const r = 6371000;
        const toRad = (deg: number): number => (deg * Math.PI) / 180;
        const dLat = toRad(b[0] - a[0]);
        const dLon = toRad(b[1] - a[1]);
        const lat1 = toRad(a[0]);
        const lat2 = toRad(b[0]);
        const sinDLat = Math.sin(dLat / 2);
        const sinDLon = Math.sin(dLon / 2);
        const h = sinDLat * sinDLat + Math.cos(lat1) * Math.cos(lat2) * sinDLon * sinDLon;
        return 2 * r * Math.asin(Math.sqrt(h));
    }
}
