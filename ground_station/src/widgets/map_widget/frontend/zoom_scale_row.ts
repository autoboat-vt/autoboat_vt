import { Control, control, DomEvent, DomUtil, type Map as LeafletMap } from "leaflet";

/**
 * Leaflet control that places the scale bar to the left of the zoom buttons in
 * a single horizontal row.
 *
 * Leaflet stacks every control in a corner vertically, so there is no built-in
 * way to put two of them side by side. This control owns both, adds them to the
 * map normally, and then moves their containers into its own row element.
 */
export class ZoomScaleRowControl extends Control {
    private readonly zoomControl = control.zoom();
    private readonly scaleControl = control.scale();

    constructor() {
        super({ position: "bottomright" });
    }

    override onAdd(map: LeafletMap): HTMLElement {
        const row = DomUtil.create("div", "zoom-scale-row");

        // keep clicks and wheel events on the row from reaching the map
        DomEvent.disableClickPropagation(row);
        DomEvent.disableScrollPropagation(row);

        this.zoomControl.addTo(map);
        this.scaleControl.addTo(map);

        // Scale first, so flex lays it out to the left of the zoom buttons.
        const scaleContainer = this.scaleControl.getContainer();
        const zoomContainer = this.zoomControl.getContainer();
        if (scaleContainer !== undefined) {
            row.append(scaleContainer);
        }
        if (zoomContainer !== undefined) {
            row.append(zoomContainer);
        }

        return row;
    }

    override onRemove(_map: LeafletMap): void {
        this.zoomControl.remove();
        this.scaleControl.remove();
    }
}
