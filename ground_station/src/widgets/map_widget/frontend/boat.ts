import { type Icon, type Map as LeafletMap, type Marker, marker } from "leaflet";
import type { LatLngTuple } from "./types";

interface BoatState {
    heading: number;
    location: LatLngTuple;
}

export class BoatManager {
    private readonly boat: BoatState = { heading: 0, location: [0, 0] };
    private marker: Marker;

    constructor(
        private readonly map: LeafletMap,
        private readonly getBoatIcon: (scale?: number) => Icon
    ) {
        this.marker = marker(this.boat.location, {
            icon: this.getBoatIcon(),
            rotationAngle: this.boat.heading,
            rotationOrigin: "center"
        }).addTo(this.map);
    }

    /**
     * Centers the map on the boat.
     */
    focus(): void {
        this.map.setView(this.boat.location, this.map.getMaxZoom());
    }

    setLocation(lat: number, lon: number): void {
        this.boat.location = [lat, lon];
        this.marker.setLatLng(this.boat.location);
    }

    getLocation(): LatLngTuple {
        return this.boat.location;
    }

    setHeading(heading: number): void {
        this.boat.heading = heading;
        this.marker.setRotationAngle(90 - heading);
    }

    getHeading(): number {
        return this.boat.heading;
    }

    /**
     * Updates both boat location and heading.
     */
    setLocationAndHeading(lat: number, lon: number, heading: number): void {
        this.setLocation(lat, lon);
        this.setHeading(heading);
    }

    /**
     * Rescales the boat icon for the given zoom level.
     */
    setScale(scale: number): void {
        this.marker.setIcon(this.getBoatIcon(scale));
    }
}
