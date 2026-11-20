import { DomUtil, type Map as LeafletMap, type Polyline, polyline } from "leaflet";

/**
 * Pane holding the graticule lines. It sits above the bathymetry pane (250) so
 * the lines stay readable over the depth bands, and below the overlay pane
 * (400) so waypoints, buoys, and measurements always draw on top.
 */
const GRID_PANE = "gridPane";
const GRID_PANE_Z_INDEX = "350";

/** Colour of the labelled graticule lines. */
const GRID_COLOR = "#33475b";

/** Opacity of the labelled graticule lines. */
const GRID_OPACITY = 0.45;

/** Weight in pixels of the labelled graticule lines. */
const GRID_WEIGHT = 1;

/**
 * Minor lines subdivide each major cell to make the grid easier to read
 * between labels. They are drawn lighter and thinner so they stay clearly
 * secondary to the labelled lines.
 */
const MINOR_GRID_OPACITY = 0.22;
const MINOR_GRID_WEIGHT = 1;

/**
 * Number of minor divisions per major cell. Four gives thirds of the labelled
 * spacing, which stays legible without cluttering the map.
 */
const MINOR_DIVISIONS = 4;

/**
 * Mercator projection limit in degrees. Leaflet clamps the map to this
 * latitude, so graticule lines must not extend past it.
 */
const MERCATOR_LIMIT = 85;

/** Maximum longitude, matching the map's own `noWrap` bounds. */
const MAX_LONGITUDE = 180;

/**
 * Candidate spacings in degrees, from coarsest to finest. Each is a value a
 * reader can divide mentally, which is what makes a graticule legible.
 */
const STEPS: readonly number[] = [30, 20, 10, 5, 2, 1, 0.5, 0.25, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001, 0.0005];

/** Target number of divisions per axis; the step closest to this wins. */
const TARGET_DIVISIONS = 6;

/**
 * Inset in pixels from the viewport edge for the edge labels.
 */
const LABEL_INSET_PX = 12;

/**
 * Width in pixels reserved at the right edge for the control stack (measure
 * tool, zoom buttons, scale bar, attribution) which lives in the bottom right
 * corner. A longitude label is centred on its line, so the rightmost label
 * would otherwise be drawn behind the controls.
 */
const LABEL_RIGHT_RESERVE_PX = 52;

/**
 * Height in pixels reserved at the top of the left edge for the longitude
 * labels. Latitude labels are vertically centred on their line, so one whose
 * line falls inside this band would be drawn on top of a longitude label:
 * a longitude label spans `[inset, inset + labelHeight]`, and a latitude label
 * spans `[y - labelHeight / 2, y + labelHeight / 2]`. With a 13px line height
 * that means any line above `12 + 13 + 13 / 2 ≈ 32` collides.
 */
const LABEL_TOP_RESERVE_PX = 32;

/**
 * Pick the tightest spacing that still yields at least {@link TARGET_DIVISIONS}
 * lines across the viewport, so density stays readable at every zoom level.
 *
 * @param spanDegrees - The visible extent along one axis, in degrees.
 */
function chooseStep(spanDegrees: number): number {
    const target = spanDegrees / TARGET_DIVISIONS;

    // STEPS runs coarse to fine; the last entry still >= target is the finest
    // grid that doesn't exceed the target line count.
    for (let i = STEPS.length - 1; i >= 0; i--) {
        const step = STEPS[i];
        if (step !== undefined && step >= target) {
            return step;
        }
    }

    return STEPS[0] ?? 30;
}

/**
 * Number of decimal places worth showing for a given spacing.
 *
 * @param step - The spacing between lines, in degrees.
 */
function decimalsFor(step: number): number {
    if (step >= 1) {
        return 0;
    }
    if (step >= 0.1) {
        return 1;
    }
    if (step >= 0.01) {
        return 2;
    }
    if (step >= 0.001) {
        return 3;
    }
    return 4;
}

/**
 * Format a latitude as a labelled graticule value, e.g. `45°N`.
 *
 * @param latitude - Latitude in degrees.
 * @param step - The spacing between lines, in degrees.
 */
function formatLatitude(latitude: number, step: number): string {
    if (Math.abs(latitude) < 1e-9) {
        return "0°";
    }

    return `${Math.abs(latitude).toFixed(decimalsFor(step))}°${latitude > 0 ? "N" : "S"}`;
}

/**
 * Format a longitude as a labelled graticule value, e.g. `120°W`.
 *
 * @param longitude - Longitude in degrees.
 * @param step - The spacing between lines, in degrees.
 */
function formatLongitude(longitude: number, step: number): string {
    if (Math.abs(longitude) < 1e-9) {
        return "0°";
    }

    return `${Math.abs(longitude).toFixed(decimalsFor(step))}°${longitude > 0 ? "E" : "W"}`;
}

/**
 * Toggleable latitude/longitude graticule.
 *
 * Lines are rebuilt from the current viewport whenever the map settles, so the
 * spacing always suits the zoom level and only visible lines are created.
 *
 * Labels are plain DOM in an absolutely positioned overlay rather than Leaflet
 * markers: markers live in the animated marker pane, which lets a label from a
 * previous redraw linger and drift while the map is zooming. Owning the label
 * elements directly means a rebuild can remove every one of them, with nothing
 * left behind.
 *
 * Because the labels are positioned in container pixels rather than by
 * Leaflet, they do not follow the map pane while it is being dragged. To avoid
 * the labels visibly lagging and snapping back on every pan, their positions are
 * refreshed on each animation frame while the map moves; the line and label set
 * is only rebuilt once the map settles, since that is when the grid spacing
 * actually needs to change.
 */
export class LatLonGridManager {
    private lines: Polyline[] = [];
    private labelLayer: HTMLElement;
    private visible = false;
    private frameHandle: number | null = null;

    /** Calls that re-position the current labels from their stored anchors. */
    private positioners: Array<() => void> = [];

    constructor(private readonly map: LeafletMap) {
        if (map.getPane(GRID_PANE) === undefined) {
            const pane = map.createPane(GRID_PANE);
            pane.style.zIndex = GRID_PANE_Z_INDEX;
        }

        // The map container is `position: relative` with `overflow: hidden`, so
        // container coordinates can be used directly for absolutely positioned
        // children. z-index 650 keeps labels above the panes but below controls.
        this.labelLayer = DomUtil.create("div", "grid-label-layer", map.getContainer());
        this.labelLayer.style.zIndex = "650";

        this.map.on("move", () => this.scheduleReposition());
        this.map.on("zoom", () => this.scheduleReposition());
        this.map.on("moveend", () => this.redraw());
        this.map.on("zoomend", () => this.redraw());
    }

    /**
     * Re-position the existing labels on the next animation frame.
     *
     * Coalesced through `requestAnimationFrame` so a burst of move events costs
     * at most one layout pass per frame.
     */
    private scheduleReposition(): void {
        if (!this.visible || this.frameHandle !== null) {
            return;
        }

        this.frameHandle = requestAnimationFrame(() => {
            this.frameHandle = null;
            for (const position of this.positioners) {
                position();
            }
        });
    }

    /**
     * Stop any pending reposition work.
     */
    private cancelReposition(): void {
        if (this.frameHandle !== null) {
            cancelAnimationFrame(this.frameHandle);
            this.frameHandle = null;
        }
    }

    /**
     * Show or hide the graticule.
     *
     * @param visible - Whether the grid should be drawn.
     */
    setVisible(visible: boolean): void {
        if (this.visible === visible) {
            return;
        }

        this.visible = visible;
        if (visible) {
            this.redraw();
        } else {
            this.clear();
        }
    }

    /**
     * Whether the graticule is currently drawn.
     */
    isVisible(): boolean {
        return this.visible;
    }

    /**
     * Remove every line and label.
     */
    private clear(): void {
        this.cancelReposition();

        for (const line of this.lines) {
            this.map.removeLayer(line);
        }
        this.lines = [];

        this.positioners = [];
        this.labelLayer.replaceChildren();
    }

    /**
     * Rebuild every line and label for the current viewport.
     */
    private redraw(): void {
        if (!this.visible) {
            return;
        }

        this.clear();

        const bounds = this.map.getBounds();
        const west = Math.max(-MAX_LONGITUDE, bounds.getWest());
        const east = Math.min(MAX_LONGITUDE, bounds.getEast());
        const south = Math.max(-MERCATOR_LIMIT, bounds.getSouth());
        const north = Math.min(MERCATOR_LIMIT, bounds.getNorth());

        if (east <= west || north <= south) {
            return;
        }

        const size = this.map.getSize();
        const lonStep = chooseStep(east - west);
        const latStep = chooseStep(north - south);

        // Minor lines first, so the labelled lines paint over them where the two
        // meet and the major grid stays visually dominant.
        this.drawMinorMeridians(west, east, south, north, lonStep);
        this.drawMinorParallels(south, north, west, east, latStep);

        this.drawMeridians(west, east, south, north, lonStep);
        this.drawParallels(south, north, west, east, latStep, size.y);
    }

    /**
     * Draw the north-south lines and label them along the top edge.
     */
    private drawMeridians(west: number, east: number, south: number, north: number, step: number): void {
        // Count iterations rather than accumulate `lon += step`, which would
        // drift for fractional steps like 0.05 and skip the final line.
        const firstIndex = Math.ceil(west / step);
        const lastIndex = Math.floor(east / step);

        for (let index = firstIndex; index <= lastIndex; index++) {
            const value = index * step;
            if (value < -MAX_LONGITUDE || value > MAX_LONGITUDE) {
                continue;
            }

            this.addLine([
                [south, value],
                [north, value]
            ]);

            this.addMeridianLabel(value, formatLongitude(value, step));
        }
    }

    /**
     * Draw the unlabelled minor meridians that subdivide each major cell.
     *
     * Only the lines strictly between two major meridians are drawn, so no line
     * is painted twice and none sits underneath a labelled one.
     *
     * @param step - Spacing between the labelled meridians, in degrees.
     */
    private drawMinorMeridians(west: number, east: number, south: number, north: number, step: number): void {
        const minorStep = step / MINOR_DIVISIONS;
        const firstIndex = Math.ceil(west / minorStep);
        const lastIndex = Math.floor(east / minorStep);

        for (let index = firstIndex; index <= lastIndex; index++) {
            if (index % MINOR_DIVISIONS === 0) {
                continue;
            }

            const value = index * minorStep;
            if (value < -MAX_LONGITUDE || value > MAX_LONGITUDE) {
                continue;
            }

            this.addLine(
                [
                    [south, value],
                    [north, value]
                ],
                true
            );
        }
    }

    /**
     * Draw the unlabelled minor parallels that subdivide each major cell.
     *
     * @param step - Spacing between the labelled parallels, in degrees.
     */
    private drawMinorParallels(south: number, north: number, west: number, east: number, step: number): void {
        const minorStep = step / MINOR_DIVISIONS;
        const firstIndex = Math.ceil(south / minorStep);
        const lastIndex = Math.floor(north / minorStep);

        for (let index = firstIndex; index <= lastIndex; index++) {
            if (index % MINOR_DIVISIONS === 0) {
                continue;
            }

            const value = index * minorStep;
            if (value < -MERCATOR_LIMIT || value > MERCATOR_LIMIT) {
                continue;
            }

            this.addLine(
                [
                    [value, west],
                    [value, east]
                ],
                true
            );
        }
    }

    /**
     * Draw the east-west lines and label them along the left edge.
     *
     * @param heightPx - Viewport height, used to keep labels on screen.
     */
    private drawParallels(south: number, north: number, west: number, east: number, step: number, heightPx: number): void {
        const firstIndex = Math.ceil(south / step);
        const lastIndex = Math.floor(north / step);

        for (let index = firstIndex; index <= lastIndex; index++) {
            const value = index * step;
            if (value < -MERCATOR_LIMIT || value > MERCATOR_LIMIT) {
                continue;
            }

            this.addLine([
                [value, west],
                [value, east]
            ]);

            const y = this.map.latLngToContainerPoint([value, 0]).y;

            // Skip the top band: the longitude labels live there, and a latitude
            // label centred on a line inside it would collide in the corner.
            if (y < LABEL_TOP_RESERVE_PX || y > heightPx - LABEL_INSET_PX) {
                continue;
            }

            this.addParallelLabel(value, formatLatitude(value, step));
        }
    }

    /**
     * Add a graticule line to the map.
     *
     * @param path - The two endpoints of the line.
     * @param minor - Whether this is an unlabelled minor line, drawn lighter.
     */
    private addLine(path: [[number, number], [number, number]], minor = false): void {
        this.lines.push(
            polyline(path, {
                pane: GRID_PANE,
                color: GRID_COLOR,
                opacity: minor ? MINOR_GRID_OPACITY : GRID_OPACITY,
                weight: minor ? MINOR_GRID_WEIGHT : GRID_WEIGHT,
                interactive: false
            }).addTo(this.map)
        );
    }

    /**
     * Place a text label for a meridian, kept on the top edge.
     *
     * The label is hidden once its line leaves the viewport, rather than being
     * pinned to the edge where several off-screen labels would stack up.
     *
     * @param longitude - Longitude of the line, in degrees.
     * @param text - The label text.
     */
    private addMeridianLabel(longitude: number, text: string): void {
        const label = DomUtil.create("div", "grid-label grid-label-top", this.labelLayer);
        label.textContent = text;
        label.style.top = `${LABEL_INSET_PX}px`;

        const position = (): void => {
            const x = this.map.latLngToContainerPoint([0, longitude]).x;
            const widthPx = this.map.getSize().x;

            // Keep clear of the control stack pinned to the right edge, where the
            // label would otherwise be drawn behind the measure and zoom buttons.
            const onScreen = x >= 0 && x <= widthPx - LABEL_RIGHT_RESERVE_PX;

            label.style.display = onScreen ? "" : "none";
            // Clamp the few pixels a label needs so it stays fully readable.
            const clampedX = Math.min(Math.max(x, LABEL_INSET_PX), widthPx - LABEL_RIGHT_RESERVE_PX);
            label.style.left = `${clampedX}px`;
        };

        position();
        this.positioners.push(position);
    }

    /**
     * Place a text label for a parallel, kept on the left edge.
     *
     * The label is hidden once its line leaves the viewport, and while it sits
     * in the top band reserved for the longitude labels, so the two axes never
     * collide in the corner.
     *
     * @param latitude - Latitude of the line, in degrees.
     * @param text - The label text.
     */
    private addParallelLabel(latitude: number, text: string): void {
        const label = DomUtil.create("div", "grid-label grid-label-left", this.labelLayer);
        label.textContent = text;
        label.style.left = `${LABEL_INSET_PX}px`;

        const position = (): void => {
            const y = this.map.latLngToContainerPoint([latitude, 0]).y;
            const heightPx = this.map.getSize().y;
            const onScreen = y >= LABEL_TOP_RESERVE_PX && y <= heightPx - LABEL_INSET_PX;

            label.style.display = onScreen ? "" : "none";
            label.style.top = `${y}px`;
        };

        position();
        this.positioners.push(position);
    }
}
