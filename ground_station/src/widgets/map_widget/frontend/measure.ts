import { distance } from "@turf/distance";
import {
    type CircleMarker,
    Control,
    circleMarker,
    DomEvent,
    type LatLng,
    type Map as LeafletMap,
    type Polyline,
    polyline
} from "leaflet";

/** Colour of the committed measurement path and its vertices. */
const MEASURE_COLOR = "#ff9f1c";

/** Colour of the rubber-band preview segment. */
const PREVIEW_COLOR = "#ffd866";

/** Radius in pixels of each measurement vertex. */
const VERTEX_RADIUS_PX = 5;

/**
 * Vertices closer together than this (in screen pixels) are treated as the
 * duplicate produced by the second click of a double click.
 */
const DUPLICATE_VERTEX_PX = 5;

/**
 * Every measurement segment and the rubber-band preview render here. It sits
 * above the overlay pane (400) so the measurement stays visible over other
 * vector layers, and below the vertex pane so lines never cover the points.
 */
const LINE_PANE = "measureLinePane";
const LINE_PANE_Z_INDEX = "440";

/**
 * Vertices render into their own pane above {@link LINE_PANE}. The map uses a
 * canvas renderer, and a canvas paints in insertion order within its pane;
 * sharing a pane with the polyline would therefore draw the first vertex under
 * the line that is created after it.
 */
const VERTEX_PANE = "measureVertexPane";
const VERTEX_PANE_Z_INDEX = "450";

/** Metres in one kilometre. */
const METERS_PER_KILOMETER = 1000;

/** Metres in one international nautical mile. */
const METERS_PER_NAUTICAL_MILE = 1852;

/**
 * Format a distance in metres using the unit that keeps the number readable.
 *
 * @param meters - The distance in metres.
 */
function formatMetric(meters: number): string {
    if (meters < METERS_PER_KILOMETER) {
        return `${meters.toFixed(1)} m`;
    }

    const kilometers = meters / METERS_PER_KILOMETER;
    return `${kilometers.toFixed(kilometers < 10 ? 2 : 1)} km`;
}

/**
 * Format a distance in metres as nautical miles.
 *
 * @param meters - The distance in metres.
 */
function formatNauticalMiles(meters: number): string {
    const nauticalMiles = meters / METERS_PER_NAUTICAL_MILE;
    return `${nauticalMiles.toFixed(nauticalMiles < 10 ? 2 : 1)} nmi`;
}

/**
 * Create the measurement panes, unless the map already has them.
 *
 * @param map - The Leaflet map to add the panes to.
 */
function createPanes(map: LeafletMap): void {
    if (map.getPane(VERTEX_PANE) !== undefined) {
        return;
    }

    const linePane = map.createPane(LINE_PANE);
    linePane.style.zIndex = LINE_PANE_Z_INDEX;

    const vertexPane = map.createPane(VERTEX_PANE);
    vertexPane.style.zIndex = VERTEX_PANE_Z_INDEX;
}

/**
 * Leaflet control holding the measure tool's toggle button and live readout.
 *
 * The readout is shown while measuring and whenever a measurement exists, so
 * the final distance stays readable after the tool has been switched off.
 */
class MeasureControl extends Control {
    private button: HTMLButtonElement | null = null;
    private panel: HTMLElement | null = null;
    private totalText: HTMLElement | null = null;
    private secondaryText: HTMLElement | null = null;
    private metaText: HTMLElement | null = null;
    private measuring = false;
    private hasContent = false;

    /**
     * @param onToggle - Called when the toggle button is pressed.
     * @param onClear - Called when the readout's clear button is pressed.
     */
    constructor(
        private readonly onToggle: () => void,
        private readonly onClear: () => void
    ) {
        // bottom right, above the zoom buttons: the left edge is reserved for
        // the latitude labels, which would otherwise be hidden behind it
        super({ position: "bottomright" });
    }

    override onAdd(map: LeafletMap): HTMLElement {
        const container = document.createElement("div");
        container.className = "leaflet-control measure-control";

        // keep clicks, drags, and wheel events on the control from reaching the map
        DomEvent.disableClickPropagation(container);
        DomEvent.disableScrollPropagation(container);

        this.button = document.createElement("button");
        this.button.type = "button";
        this.button.className = "measure-toggle";
        this.button.title = "Measure distance (M)";
        this.button.setAttribute("aria-label", "Measure distance");
        this.button.innerHTML = `
            <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <rect x="2" y="8" width="20" height="8" rx="1" />
                <path d="M6 8v3M10 8v3M14 8v3M18 8v3" />
            </svg>`;
        this.button.addEventListener("click", () => this.onToggle());
        container.append(this.button);

        // The readout is a separate overlay anchored to the bottom centre of the
        // map rather than part of this corner control, so it stays centred and
        // never covers the controls or the latitude labels.
        this.panel = document.createElement("div");
        this.panel.className = "measure-panel";
        this.panel.innerHTML = `
            <div class="measure-values">
                <div class="measure-total">0 m</div>
                <div class="measure-secondary">0 nmi</div>
            </div>
            <div class="measure-meta">No points yet</div>
            <div class="measure-hint">
                Left click to add a point, right click to undo the last point, Esc to finish.
            </div>
            <button class="measure-clear" type="button">Clear</button>`;

        DomEvent.disableClickPropagation(this.panel);
        DomEvent.disableScrollPropagation(this.panel);
        map.getContainer().append(this.panel);

        this.totalText = this.panel.querySelector<HTMLElement>(".measure-total");
        this.secondaryText = this.panel.querySelector<HTMLElement>(".measure-secondary");
        this.metaText = this.panel.querySelector<HTMLElement>(".measure-meta");

        const clearButton = this.panel.querySelector<HTMLButtonElement>(".measure-clear");
        clearButton?.addEventListener("click", () => this.onClear());

        this.updateVisibility();
        return container;
    }

    override onRemove(_map: LeafletMap): void {
        // the panel lives in the map container, not in this control's element,
        // so Leaflet will not clean it up for us
        this.panel?.remove();

        this.button = null;
        this.panel = null;
        this.totalText = null;
        this.secondaryText = null;
        this.metaText = null;
    }

    /**
     * Highlight the toggle button while the tool is capturing clicks.
     */
    setMeasuring(measuring: boolean): void {
        this.measuring = measuring;
        this.button?.classList.toggle("active", measuring);
        this.updateVisibility();
    }

    /**
     * Update the readout with the current measurement.
     *
     * @param totalMeters - Total measured distance in metres.
     * @param pointCount - Number of placed vertices.
     * @param lastLegMeters - Length of the most recent leg, or `null` if there is none.
     */
    setReadout(totalMeters: number, pointCount: number, lastLegMeters: number | null): void {
        this.hasContent = pointCount > 0;

        if (this.totalText) {
            this.totalText.textContent = formatMetric(totalMeters);
        }
        if (this.secondaryText) {
            this.secondaryText.textContent = formatNauticalMiles(totalMeters);
        }
        if (this.metaText) {
            const legs = Math.max(0, pointCount - 1);
            let meta = pointCount === 1 ? "1 point" : `${pointCount} points`;

            if (legs > 0) {
                meta += ` · ${legs === 1 ? "1 leg" : `${legs} legs`}`;
            }
            if (lastLegMeters !== null) {
                meta += ` · last ${formatMetric(lastLegMeters)}`;
            }

            this.metaText.textContent = meta;
        }

        this.updateVisibility();
    }

    private updateVisibility(): void {
        if (this.panel) {
            this.panel.hidden = !(this.measuring || this.hasContent);
        }
    }
}

/**
 * Interactive distance measuring tool drawn directly on the map.
 *
 * Left clicks place vertices, right clicks undo the last vertex, and Esc leaves
 * measure mode while keeping the finished path on screen. Distances are
 * great-circle lengths computed with Turf, reported in both metric and
 * nautical units.
 */
export class MeasureManager {
    private readonly control: MeasureControl;
    private points: LatLng[] = [];
    private vertices: CircleMarker[] = [];
    private line: Polyline | null = null;
    private preview: Polyline | null = null;
    private hover: LatLng | null = null;
    private totalMeters = 0;
    private active = false;

    constructor(private readonly map: LeafletMap) {
        createPanes(map);

        this.control = new MeasureControl(
            () => this.toggle(),
            () => this.clear()
        );
        this.control.addTo(map);

        // Listening on the container (rather than via Leaflet's synthetic events)
        // lets measuring work anywhere on the map, including over markers, which
        // swallow Leaflet click events. The two guards below restore what
        // Leaflet's own click handling would otherwise do: skip clicks on map
        // controls, and skip the click that ends a pan drag.
        const container = map.getContainer();

        // Leaflet swallows the click that ends a pan drag or a box zoom. Because
        // we listen on the container directly, that suppression is ours to do:
        // remember when a drag finished, and skip the click that follows it.
        let dragJustEnded = false;
        map.on("dragend", () => {
            dragJustEnded = true;
        });
        map.on("boxzoomend", () => {
            dragJustEnded = true;
        });

        container.addEventListener("mousedown", () => {
            dragJustEnded = false;
        });

        container.addEventListener("click", (event: MouseEvent) => {
            if (!this.active) {
                return;
            }

            if (event.target instanceof Element && event.target.closest(".leaflet-control") !== null) {
                return;
            }

            if (dragJustEnded) {
                dragJustEnded = false;
                return;
            }

            this.addPoint(map.mouseEventToLatLng(event));
        });

        container.addEventListener("contextmenu", (event: MouseEvent) => {
            if (!this.active) {
                return;
            }

            event.preventDefault();
            this.removeLastPoint();
        });

        // Double click finishes the measurement. The second click has already
        // added a near-duplicate vertex, so drop it before finishing.
        container.addEventListener("dblclick", (event: MouseEvent) => {
            if (!this.active) {
                return;
            }

            if (event.target instanceof Element && event.target.closest(".leaflet-control") !== null) {
                return;
            }

            event.preventDefault();
            this.removeDuplicateTail();
            this.finish();
        });

        container.addEventListener("mousemove", (event: MouseEvent) => {
            if (!this.active) {
                return;
            }

            this.hover = map.mouseEventToLatLng(event);
            this.updatePreview();
            this.refreshReadout();
        });

        container.addEventListener("mouseleave", () => {
            if (this.hover === null) {
                return;
            }

            this.hover = null;
            this.clearPreview();
            this.refreshReadout();
        });

        document.addEventListener("keydown", (event: KeyboardEvent) => {
            if (event.key === "Escape" && this.active) {
                this.finish();
            }
        });
    }

    /**
     * Whether the tool is currently capturing map clicks.
     */
    isMeasuring(): boolean {
        return this.active;
    }

    /**
     * Toggle between measuring and finished. Starting a measurement discards
     * any previous path so each run is independent.
     */
    toggle(): void {
        if (this.active) {
            this.finish();
            return;
        }

        this.clear();
        this.active = true;
        this.map.doubleClickZoom.disable();
        this.control.setMeasuring(true);
        this.map.getContainer().style.cursor = "crosshair";
        this.refreshReadout();
    }

    /**
     * Leave measure mode, keeping the measured path and readout on the map.
     */
    finish(): void {
        if (!this.active) {
            return;
        }

        this.active = false;
        this.hover = null;
        this.clearPreview();
        this.map.doubleClickZoom.enable();
        this.map.getContainer().style.cursor = "";
        this.control.setMeasuring(false);
        this.refreshReadout();
    }

    /**
     * Remove every vertex, segment, and the readout's contents.
     */
    clear(): void {
        for (const vertex of this.vertices) {
            this.map.removeLayer(vertex);
        }
        this.vertices = [];
        this.points = [];
        this.totalMeters = 0;

        if (this.line !== null) {
            this.map.removeLayer(this.line);
            this.line = null;
        }

        this.clearPreview();
        this.control.setReadout(0, 0, null);
    }

    /**
     * Great-circle distance between two points in metres, via Turf.
     */
    private segmentMeters(a: LatLng, b: LatLng): number {
        return distance([a.lng, a.lat], [b.lng, b.lat], { units: "kilometers" }) * METERS_PER_KILOMETER;
    }

    private addPoint(latlng: LatLng): void {
        const previous = this.points[this.points.length - 1];
        this.points.push(latlng);

        this.vertices.push(
            circleMarker(latlng, {
                pane: VERTEX_PANE,
                radius: VERTEX_RADIUS_PX,
                color: MEASURE_COLOR,
                weight: 2,
                fillColor: "#ffffff",
                fillOpacity: 1,
                interactive: false
            }).addTo(this.map)
        );

        if (previous !== undefined) {
            this.totalMeters += this.segmentMeters(previous, latlng);
        }

        if (this.line === null) {
            this.line = polyline(this.points, {
                pane: LINE_PANE,
                color: MEASURE_COLOR,
                weight: 3,
                opacity: 0.9,
                interactive: false
            }).addTo(this.map);
        } else {
            this.line.setLatLngs(this.points);
        }

        this.updatePreview();
        this.refreshReadout();
    }

    private removeLastPoint(): void {
        if (this.points.length === 0) {
            return;
        }

        this.dropTail();
        this.recompute();
        this.updatePreview();
        this.refreshReadout();
    }

    /**
     * Drop the final vertex and its layer, without recomputing totals.
     */
    private dropTail(): void {
        this.points.pop();
        const vertex = this.vertices.pop();
        if (vertex !== undefined) {
            this.map.removeLayer(vertex);
        }

        if (this.points.length === 0 && this.line !== null) {
            this.map.removeLayer(this.line);
            this.line = null;
        }
    }

    /**
     * Remove the last vertex when a double click left a near-duplicate of the
     * point before it, so finishing a measurement doesn't add a stray vertex.
     * Compared in screen space, so it behaves the same at every zoom level.
     */
    private removeDuplicateTail(): void {
        if (this.points.length < 2) {
            return;
        }

        const last = this.points[this.points.length - 1];
        const previous = this.points[this.points.length - 2];
        if (last === undefined || previous === undefined) {
            return;
        }

        const from = this.map.latLngToContainerPoint(previous);
        const to = this.map.latLngToContainerPoint(last);
        if (from.distanceTo(to) < DUPLICATE_VERTEX_PX) {
            this.dropTail();
        }
    }

    /**
     * Recompute the committed total after a vertex was removed.
     */
    private recompute(): void {
        this.totalMeters = 0;
        for (let i = 1; i < this.points.length; i++) {
            const a = this.points[i - 1];
            const b = this.points[i];
            if (a !== undefined && b !== undefined) {
                this.totalMeters += this.segmentMeters(a, b);
            }
        }

        this.line?.setLatLngs(this.points);
    }

    /**
     * Length of the most recently placed leg, or `null` if there is none.
     */
    private lastLegMeters(): number | null {
        if (this.points.length < 2) {
            return null;
        }

        const a = this.points[this.points.length - 2];
        const b = this.points[this.points.length - 1];
        if (a === undefined || b === undefined) {
            return null;
        }

        return this.segmentMeters(a, b);
    }

    /**
     * Draw the dashed segment from the last vertex to the cursor.
     */
    private updatePreview(): void {
        const last = this.points[this.points.length - 1];

        if (!this.active || last === undefined || this.hover === null) {
            this.clearPreview();
            return;
        }

        if (this.preview === null) {
            this.preview = polyline([last, this.hover], {
                pane: LINE_PANE,
                color: PREVIEW_COLOR,
                weight: 2,
                dashArray: "6 6",
                opacity: 0.9,
                interactive: false
            }).addTo(this.map);
        } else {
            this.preview.setLatLngs([last, this.hover]);
        }
    }

    private clearPreview(): void {
        if (this.preview !== null) {
            this.map.removeLayer(this.preview);
            this.preview = null;
        }
    }

    /**
     * Push the current totals into the readout, including the live preview leg
     * while the cursor is on the map.
     */
    private refreshReadout(): void {
        const last = this.points[this.points.length - 1];

        if (this.active && last !== undefined && this.hover !== null) {
            const previewLeg = this.segmentMeters(last, this.hover);
            this.control.setReadout(this.totalMeters + previewLeg, this.points.length, previewLeg);
            return;
        }

        this.control.setReadout(this.totalMeters, this.points.length, this.lastLegMeters());
    }
}
