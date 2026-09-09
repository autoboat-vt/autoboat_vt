import { type GeoJSON as GeoJSONLayer, type GeoJSONOptions, geoJSON, type Map as LeafletMap, type PathOptions } from "leaflet";

/*
 * Minimal GeoJSON typings for what the backend serves, kept local so the
 * optional `geojson` type package is not a hard dependency. The boundary is a
 * single (Multi)Polygon with no meaningful properties.
 */
type GeoJsonGeometry = Record<string, unknown>;

type BoundaryFeature = {
    type: "Feature";
    geometry: GeoJsonGeometry;
    properties?: Record<string, unknown>;
};

type BoundaryCollection = {
    type: "FeatureCollection";
    features: BoundaryFeature[];
};

/** Stroke used for the faint ocean/land boundary outline. */
const BOUNDARY_STYLE: PathOptions = {
    fill: false,
    color: "red",
    weight: 5,
    opacity: 1
};

/**
 * Toggleable Leaflet overlay rendering a faint outline of the Natural Earth
 * ocean polygon used by the land checker, served by the map backend as GeoJSON.
 *
 * The layer is fetch-on-demand: it is not downloaded until first enabled, and
 * after that the polygon set is kept in memory so the map_features toggle is
 * instant. It is non-interactive so clicks still fall through to waypoint
 * placement, and renders into the shared low `bathyPane` under labels and hits.
 */
export class LandBoundaryManager {
    private layer: GeoJSONLayer | null = null;
    private fetching = false;

    constructor(
        private readonly map: LeafletMap,
        private readonly url: string
    ) {}

    /**
     * Load and add (or remove) the boundary layer.
     *
     * The first visible call fetches the GeoJSON; later toggles reuse the
     * cached polygons.
     *
     * @param visible - Whether the layer should be shown.
     */
    async setVisible(visible: boolean): Promise<void> {
        if (!visible) {
            if (this.layer !== null) {
                this.map.removeLayer(this.layer);
            }
            return;
        }

        if (this.layer !== null) {
            this.layer.addTo(this.map);
            return;
        }

        if (this.fetching) {
            return;
        }

        this.fetching = true;
        try {
            const response = await fetch(this.url);
            if (!response.ok) {
                console.warn(`Land boundary unavailable (HTTP ${response.status})`);
                return;
            }

            const data = (await response.json()) as BoundaryCollection;

            this.layer = geoJSON(data as GeoJSON.GeoJsonObject, this.buildOptions());
            this.layer.addTo(this.map);
        } catch (error) {
            console.error("Failed to load land boundary:", error);
        } finally {
            this.fetching = false;
        }
    }

    /**
     * Leaflet GeoJSON options: render only a faint, non-interactive boundary
     * stroke into the shared low pane so clicks pass through to waypoints.
     */
    private buildOptions(): GeoJSONOptions {
        return {
            pane: "bathyPane",
            interactive: false,
            bubblingMouseEvents: false,
            style: (): PathOptions => BOUNDARY_STYLE
        };
    }
}
