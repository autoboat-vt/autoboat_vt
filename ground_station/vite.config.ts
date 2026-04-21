import { defineConfig } from "vite";
import { resolve } from "node:path";

export default defineConfig({
    root: resolve(__dirname, "src/widgets/map_widget"),
    publicDir: resolve(__dirname, "app_data"),
    server: { host: "127.0.0.1", port: 5173, strictPort: true, hmr: false }
});
