/// <reference types="vite/client" />

declare module "leaflet/dist/leaflet.css";

interface ImportMetaEnv {
    /** Map callback server port (Python ThreadingHTTPServer for waypoints/land/bathymetry). Injected by vite.config.ts from ground_station/server_ports.env. */
    readonly MAP_CALLBACK_PORT?: string;
    /** Map viewer (Vite dev server) port. Injected by vite.config.ts from ground_station/server_ports.env. */
    readonly MAP_VIEWER_PORT?: string;
    /** Static asset server port (Python TCPServer for icons/audio). Injected by vite.config.ts from ground_station/server_ports.env. */
    readonly ASSET_SERVER_PORT?: string;
}
