import { resolve } from "node:path";
import { defineConfig } from "vite";

export default defineConfig({
    root: resolve(__dirname, "src/widgets/map_widget/frontend"),
    publicDir: resolve(__dirname, "app_data"),
    server: { host: "127.0.0.1", port: 5173, strictPort: true, hmr: false }
});
