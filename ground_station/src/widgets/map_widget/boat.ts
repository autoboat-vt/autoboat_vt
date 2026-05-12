import { type Icon, type Map as LeafletMap, type Marker, marker } from "leaflet";
import type { LatLngTuple } from "./types";

interface BoatState {
    heading: number;
    location: LatLngTuple;
}

export class BoatManager {
    readonly boat: BoatState = { heading: 0, location: [0, 0] };
    readonly marker: Marker;

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

    /**
     * Updates the boat location.
     */
    updateLocation(lat: number, lon: number): void {
        this.boat.location = [lat, lon];
        this.marker.setLatLng(this.boat.location);
    }

    /**
     * Updates the boat heading.
     */
    updateHeading(heading: number): void {
        this.boat.heading = heading;
        this.marker.setRotationAngle(90 - heading);
    }

    /**
     * Updates both boat location and heading.
     */
    updateLocationAndHeading(lat: number, lon: number, heading: number): void {
        this.updateLocation(lat, lon);
        this.updateHeading(heading);
    }

    /**
     * Rescales the boat icon for the given zoom level.
     */
    setScale(scale: number): void {
        this.marker.setIcon(this.getBoatIcon(scale));
    }
}
