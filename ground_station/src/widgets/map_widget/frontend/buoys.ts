import { MarkerManager } from "./marker";

export class BuoyManager extends MarkerManager {
    static readonly defaultColor = "orange";

    add(lat: number, lon: number): void {
        this.addPoint(lat, lon, BuoyManager.defaultColor);
    }

    remove(index: number): void {
        this.removePoint(index);
    }

    clear(): void {
        this.clearPoints();
    }
}
