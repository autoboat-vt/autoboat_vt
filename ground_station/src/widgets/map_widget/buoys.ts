import {
    marker,
    type Icon,
    type Map as LeafletMap,
    type Marker
} from "leaflet";
import type { LatLngTuple } from "./types";

export class BuoyManager {
    readonly buoys: LatLngTuple[] = [];
    readonly markers = new Map<string, Marker>();

    constructor(
        private readonly map: LeafletMap,
        private readonly getIcon: (color: string) => Icon,
        private readonly buoyColor: string
    ) {}

    private makeKey(lat: number, lon: number): string {
        return `${lat.toFixed(6)},${lon.toFixed(6)}`;
    }

    /**
     * Adds a buoy unless one already exists at the same coordinates.
     */
    add(lat: number, lon: number): void {
        const key = this.makeKey(lat, lon);

        if (this.markers.has(key)) {
            return;
        }

        this.buoys.push([lat, lon]);

        const buoyMarker = marker([lat, lon], {
            icon: this.getIcon(this.buoyColor)
        }).addTo(this.map);

        this.markers.set(key, buoyMarker);
    }

    /**
     * Removes the buoy at the given index.
     */
    remove(index: number): void {
        if (index < 0 || index >= this.buoys.length) {
            return;
        }

        const [lat, lon] = this.buoys.splice(index, 1)[0]!;
        const key = this.makeKey(lat, lon);
        const buoyMarker = this.markers.get(key);

        if (buoyMarker) {
            this.map.removeLayer(buoyMarker);
            this.markers.delete(key);
        }
    }

    /**
     * Removes all buoys.
     */
    clear(): void {
        this.markers.forEach(buoyMarker => {
            this.map.removeLayer(buoyMarker);
        });

        this.markers.clear();
        this.buoys.length = 0;
    }

    /**
     * Returns the buoy marker at the given coordinates.
     */
    getMarker(lat: number, lon: number): Marker | undefined {
        return this.markers.get(this.makeKey(lat, lon));
    }
}
