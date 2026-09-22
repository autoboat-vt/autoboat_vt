import {
    type GeoJSON as GeoJSONLayer,
    type GeoJSONOptions,
    geoJSON,
    type Layer,
    type Map as LeafletMap,
    type PathOptions
} from "leaflet";

/** Fixed fill opacity applied to every depth band, independent of zoom. */
const FILL_OPACITY = 0.45;

/*
 * Minimal GeoJSON typings for what the backend serves, kept local so the
 * optional `geojson` type package is not a hard dependency. Each polygon
 * carries a `depth` property alongside its geometry.
 */
type DepthProperties = {
    /** Depth in metres (negative, following bathymetric convention). */
    depth?: number;
};

type GeoJsonGeometry = Record<string, unknown>;

type BathymetryFeature = {
    type: "Feature";
    geometry: GeoJsonGeometry;
    properties?: DepthProperties;
};

type BathymetryCollection = {
    type: "FeatureCollection";
    features: BathymetryFeature[];
};

/**
 * Bathymetric colour ramp, from shallow (light cyan) to deep (dark navy).
 * Fixed stops keyed by depth; intermediate depths are interpolated.
 */
const DEPTH_RAMP: ReadonlyArray<readonly [number, string]> = [
    [0, "#d7f0f7"],
    [200, "#a2d4ec"],
    [1000, "#6db3da"],
    [2000, "#3d8ec5"],
    [3000, "#2a6fad"],
    [4000, "#1d5494"],
    [5000, "#134078"],
    [6000, "#0b2f5d"],
    [8000, "#06203f"],
    [10000, "#031224"]
];

/**
 * Blend two #rrggbb colours.
 *
 * @param a - First colour.
 * @param b - Second colour.
 * @param t - Blend factor in [0, 1]; 0 returns `a`, 1 returns `b`.
 */
function mixHex(a: string, b: string, t: number): string {
    const clamp = Math.max(0, Math.min(1, t));
    const pa = parseInt(a.slice(1), 16);
    const pb = parseInt(b.slice(1), 16);
    const ch = (shift: number): number => {
        const ca = (pa >> shift) & 0xff;
        const cb = (pb >> shift) & 0xff;
        return Math.round(ca + (cb - ca) * clamp);
    };
    const to2 = (n: number): string => n.toString(16).padStart(2, "0");
    return `#${to2(ch(16))}${to2(ch(8))}${to2(ch(0))}`;
}

/**
 * Map a depth (negative metres) to a hex colour on the bathymetric ramp.
 *
 * @param depth - Depth in metres (negative). Positive/zero values are clamped
 *   to the shallow end of the ramp.
 */
function depthColor(depth: number): string {
    const d = Math.max(0, -depth);
    const last = DEPTH_RAMP[DEPTH_RAMP.length - 1];
    if (last === undefined || d >= last[0]) {
        return last?.[1] ?? DEPTH_RAMP[0]?.[1] ?? "#031224";
    }
    for (let i = 0; i < DEPTH_RAMP.length - 1; i++) {
        const a = DEPTH_RAMP[i];
        const b = DEPTH_RAMP[i + 1];
        if (a === undefined || b === undefined) {
            continue;
        }
        if (d >= a[0] && d <= b[0]) {
            const t = (d - a[0]) / (b[0] - a[0]);
            return mixHex(a[1], b[1], t);
        }
    }
    return DEPTH_RAMP[0]?.[1] ?? "#031224";
}

/**
 * Toggleable Leaflet overlay rendering the ocean floor depth bands served by
 * the map backend as filled GeoJSON polygons.
 *
 * The layer is fetch-on-demand: it is not downloaded until first enabled, and
 * after that the polygon set is kept in memory so the map_features toggle is
 * instant. All bands use a fixed fill opacity so appearance stays consistent
 * as the user zooms, rather than re-fading on every zoom change.
 */
export class BathymetryManager {
    private layer: GeoJSONLayer | null = null;
    private fetching = false;

    constructor(
        private readonly map: LeafletMap,
        private readonly url: string
    ) {}

    /**
     * Load and add (or remove) the depth layer.
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
                console.warn(`Bathymetry unavailable (HTTP ${response.status})`);
                return;
            }

            const data = (await response.json()) as BathymetryCollection;

            this.layer = geoJSON(data as GeoJSON.GeoJsonObject, this.buildOptions());
            this.layer.addTo(this.map);
        } catch (error) {
            console.error("Failed to load bathymetry:", error);
        } finally {
            this.fetching = false;
        }
    }

    /**
     * Leaflet GeoJSON options: style each band by depth and stay non-interactive
     * so clicks pass through to waypoint placement.
     */
    private buildOptions(): GeoJSONOptions {
        return {
            // Render into the dedicated bathymetry pane so the layer sits below
            // the default overlay pane and doesn't hide base-tile labels.
            pane: "bathyPane",
            interactive: false,
            bubblingMouseEvents: false,
            style: (feature?: GeoJSON.Feature): PathOptions => {
                const depth = (feature?.properties as DepthProperties | undefined)?.depth ?? 0;
                const color = depthColor(depth);
                return {
                    fillColor: color,
                    fillOpacity: FILL_OPACITY,
                    color,
                    weight: 0.5,
                    opacity: 0.4
                };
            },
            onEachFeature: (feature: GeoJSON.Feature, layer: Layer) => {
                const depth = (feature.properties as DepthProperties | undefined)?.depth;
                if (typeof depth === "number") {
                    layer.bindTooltip(`Depth ≈ ${-depth} m`, { sticky: true });
                }
            }
        };
    }
}
