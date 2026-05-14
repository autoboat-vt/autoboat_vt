import { type Icon, type LatLngExpression, type Map as LeafletMap, latLng, type Marker, marker } from "leaflet";

import type { LatLngTuple } from "./types";

export abstract class MarkerManager {
    readonly points: LatLngTuple[] = [];
    readonly markers = new Map<string, Marker>();
    readonly markerColors = new Map<string, string>();

    constructor(
        protected readonly map: LeafletMap,
        protected readonly getIcon: (color: string) => Icon
    ) {}

    findClosestIndex(lat: number, lon: number): number {
        let closestIndex = -1;
        let closestDistance = Number.POSITIVE_INFINITY;
        const target = latLng(lat, lon);

        this.points.forEach((point, index) => {
            const distance = target.distanceTo(latLng(point[0], point[1]));

            if (distance < closestDistance) {
                closestIndex = index;
                closestDistance = distance;
            }
        });

        return closestIndex;
    }

    getMarker(lat: number, lon: number): Marker | undefined {
        return this.markers.get(this.makeKey(lat, lon));
    }

    protected makeKey(lat: number, lon: number): string {
        return `${lat * lon}`;
    }

    protected addPoint(lat: number, lon: number, color: string): void {
        const key = this.makeKey(lat, lon);
        if (this.markers.has(key)) {
            return;
        }

        this.points.push([lat, lon]);
        const pointMarker = marker([lat, lon] satisfies LatLngExpression, {
            icon: this.getIcon(color)
        }).addTo(this.map);

        this.markers.set(key, pointMarker);
        this.markerColors.set(key, color);
        this.afterChange();
    }

    protected removePoint(index: number): void {
        if (index < 0 || index >= this.points.length) {
            return;
        }

        const point = this.points.splice(index, 1)[0];

        if (!point) {
            return;
        }

        const [lat, lon] = point;
        const key = this.makeKey(lat, lon);
        const pointMarker = this.markers.get(key);

        if (pointMarker) {
            this.map.removeLayer(pointMarker);
            this.markers.delete(key);
        }

        this.markerColors.delete(key);
        this.afterPointRemoved(key);
        this.afterChange();
    }

    protected clearPoints(): void {
        this.markers.forEach((pointMarker) => {
            this.map.removeLayer(pointMarker);
        });

        this.points.length = 0;
        this.markers.clear();
        this.markerColors.clear();
        this.afterClear();
        this.afterChange();
    }

    protected recolorPoints(color: string): void {
        this.markers.forEach((pointMarker, key) => {
            this.markerColors.set(key, color);
            pointMarker.setIcon(this.getIcon(this.getDisplayColor(key, color)));
        });
    }

    protected getDisplayColor(key: string, storedColor: string): string {
        return storedColor;
    }

    protected afterPointRemoved(_key: string): void {}

    protected afterClear(): void {}

    protected afterChange(): void {}
}
