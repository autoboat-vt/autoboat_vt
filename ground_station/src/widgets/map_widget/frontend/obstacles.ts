import {
    Control,
    type FeatureGroup,
    featureGroup,
    type GeoJSON as GeoJSONLayer,
    geoJSON,
    type LatLng,
    type Layer,
    type Map as LeafletMap,
    type LeafletMouseEvent,
    type PathOptions
} from "leaflet";
import "leaflet-draw";

/** Style used for obstacle polygons. */
const OBSTACLE_STYLE: PathOptions = {
    fill: true,
    fillColor: "#ff7800",
    fillOpacity: 0.25,
    color: "#ff7800",
    weight: 2
};

/** Screen-pixel radius within which a right-click counts as hitting the last vertex. */
const VERTEX_REMOVE_DISTANCE_PX = 15;

/**
 * Minimal view of a Leaflet.Draw polyline/polygon handler.
 *
 * Leaflet.Draw never exposes the active handler publicly, so the obstacle
 * manager reaches into the control's (undocumented) ``_toolbars`` to find it.
 * Only the members needed for right-click vertex removal are declared.
 */
interface LeafletDrawHandler {
    _markers?: { getLatLng(): LatLng }[];
    deleteLastVertex(): void;
}

/**
 * Manages the obstacle polygons drawn on the map with the Leaflet.Draw control.
 *
 * All drawn polygons live in a single feature group, which is the source of
 * truth for "the obstacles the user has authored". The group is synced back to
 * the local callback server (exactly like waypoints) whenever it changes, so the
 * Python side can read it, list it in a table, and send it to the telemetry
 * server as a GeoJSON document.
 */
export class ObstacleManager {
    private readonly drawnItems: FeatureGroup;
    private drawControl: Control.Draw | null = null;
    private drawEnabled = false;
    private visible = true;

    constructor(
        private readonly map: LeafletMap,
        private readonly syncObstacles: (featureCollection: object) => Promise<void>
    ) {
        this.drawnItems = featureGroup().addTo(this.map);

        this.map.on("draw:created", (event) => {
            const created = event as unknown as { layer: Layer };
            this.drawnItems.addLayer(created.layer);
            this.afterChange();
        });
        this.map.on("draw:edited", () => this.afterChange());
        this.map.on("draw:deleted", () => this.afterChange());

        // Right-click removes the most recently placed vertex while a polygon is
        // being drawn. The listener is only bound between drawstart and drawstop so
        // it can never interfere with the map's normal right-click behaviour.
        this.map.on("draw:drawstart", () => {
            this.map.on("contextmenu", this.onDrawContextMenu, this);
        });
        this.map.on("draw:drawstop", () => {
            this.map.off("contextmenu", this.onDrawContextMenu, this);
        });
    }

    /**
     * Shows or hides the drawn obstacle polygons.
     *
     * Hiding the layer also disables the draw control, since editing an
     * invisible polygon set would be confusing.
     *
     * @param visible - Whether the obstacles should be shown.
     */
    setVisible(visible: boolean): void {
        this.visible = visible;

        if (visible) {
            this.drawnItems.addTo(this.map);
        } else {
            this.setDrawEnabled(false);
            this.map.removeLayer(this.drawnItems);
        }
    }

    /**
     * Enables or disables the polygon draw/edit control.
     *
     * While enabled, the map's normal left-click behaviour is still under the
     * control's own handling; waypoint placement is suppressed by the caller
     * (see ``MapInterface.handleMapClick``). Waypoint removal on right-click is
     * suppressed by the caller too (see the ``contextmenu`` handler in
     * ``MapInterface``); while a polygon is actually being drawn this manager
     * turns a right-click near the last vertex into a vertex removal instead.
     *
     * @param enabled - Whether drawing should be enabled.
     */
    setDrawEnabled(enabled: boolean): void {
        if (enabled === this.drawEnabled) {
            return;
        }

        this.drawEnabled = enabled;

        if (enabled) {
            if (!this.visible) {
                this.setVisible(true);
            }
            this.drawControl = new Control.Draw(this.buildDrawOptions());
            this.map.addControl(this.drawControl);
        } else if (this.drawControl !== null) {
            this.map.removeControl(this.drawControl);
            this.drawControl = null;
        }
    }

    /** Whether the draw control is currently enabled. */
    isDrawEnabled(): boolean {
        return this.drawEnabled;
    }

    /**
     * Replaces the drawn obstacles with the polygons described by a GeoJSON document.
     *
     * @param geojsonString - A GeoJSON ``FeatureCollection`` (or single feature) as a string.
     */
    loadGeoJSONString(geojsonString: string): void {
        this.drawnItems.clearLayers();

        try {
            const parsed = JSON.parse(geojsonString) as unknown as Parameters<typeof geoJSON>[0];
            const layer = geoJSON(parsed, { style: () => OBSTACLE_STYLE }) as GeoJSONLayer;
            layer.eachLayer((child) => this.drawnItems.addLayer(child));
        } catch (error) {
            console.error("Failed to load obstacle GeoJSON:", error);
        }

        this.afterChange();
    }

    /** Removes every drawn obstacle. */
    clear(): void {
        this.drawnItems.clearLayers();
        this.afterChange();
    }

    /** The number of drawn obstacle polygons. */
    count(): number {
        return this.drawnItems.getLayers().length;
    }

    /**
     * Remove the last drawn vertex when the user right-clicks on it.
     *
     * Right-clicks are only treated as a vertex removal when they land close to
     * the most recently placed vertex; anywhere else they do nothing. This keeps
     * the behaviour predictable and prevents an accidental full-shape undo.
     */
    private onDrawContextMenu(event: LeafletMouseEvent): void {
        const handler = this.activeDrawHandler();
        const markers = handler?._markers;
        if (!handler || !markers || markers.length === 0) {
            return;
        }

        const lastMarker = markers[markers.length - 1];
        if (!lastMarker) {
            return;
        }

        const clickPoint = this.map.latLngToContainerPoint(event.latlng);
        const lastPoint = this.map.latLngToContainerPoint(lastMarker.getLatLng());

        if (clickPoint.distanceTo(lastPoint) <= VERTEX_REMOVE_DISTANCE_PX) {
            handler.deleteLastVertex();
        }
    }

    /**
     * The Leaflet.Draw handler for the toolbar mode that is currently active.
     *
     * @returns The active handler, or ``null`` when no draw mode is running.
     */
    private activeDrawHandler(): LeafletDrawHandler | null {
        if (this.drawControl === null) {
            return null;
        }

        const toolbars = (
            this.drawControl as unknown as {
                _toolbars?: Record<string, { _activeMode?: { handler?: LeafletDrawHandler } }>;
            }
        )._toolbars;

        return toolbars?.draw?._activeMode?.handler ?? null;
    }

    private buildDrawOptions(): Control.DrawConstructorOptions {
        return {
            position: "topleft",
            draw: {
                polyline: false,
                rectangle: false,
                circle: false,
                circlemarker: false,
                marker: false,
                polygon: { allowIntersection: false, shapeOptions: OBSTACLE_STYLE }
            },
            edit: { featureGroup: this.drawnItems, remove: true }
        };
    }

    private afterChange(): void {
        void this.syncObstacles(this.drawnItems.toGeoJSON() as object);
    }
}
