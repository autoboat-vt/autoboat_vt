/// <reference types="vite/client" />

declare module "leaflet/dist/leaflet.css";

interface ImportMetaEnv {
    /** Map backend server port (Python ThreadingHTTPServer for waypoints). Injected by vite.config.ts from ground_station/server_ports.env. */
    readonly MAP_SERVER_PORT?: string;
    /** Static asset server port (Python TCPServer for icons/audio). Injected by vite.config.ts from ground_station/server_ports.env. */
    readonly ASSET_SERVER_PORT?: string;
}
