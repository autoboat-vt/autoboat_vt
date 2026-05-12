import type { Icon, Map as LeafletMap } from "leaflet";
import { MarkerManager } from "./marker";

export class BuoyManager extends MarkerManager {
    readonly buoys = this.points;

    constructor(
        map: LeafletMap,
        getIcon: (color: string) => Icon,
        private readonly buoyColor: string
    ) {
        super(map, getIcon);
    }

    add(lat: number, lon: number): void {
        this.addPoint(lat, lon, this.buoyColor);
    }

    remove(index: number): void {
        this.removePoint(index);
    }

    clear(): void {
        this.clearPoints();
    }
}
