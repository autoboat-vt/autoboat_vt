import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { defineConfig } from "vite";

/**
 * Parse a KEY=VALUE env file (like `server_ports.env`) into a plain object.
 *
 * Vite's built-in `loadEnv` only reads files named `.env*`, so we parse
 * `server_ports.env` ourselves. Comments (`#`) and blank lines are skipped;
 * values are trimmed but not unquoted (our file has no quoted values).
 */
function parseEnvFile(filePath: string): Record<string, string> {
    let contents = "";
    try {
        contents = readFileSync(filePath, "utf8");
    } catch {
        return {};
    }

    const env: Record<string, string> = {};
    for (const line of contents.split("\n")) {
        const trimmed = line.trim();
        if (trimmed === "" || trimmed.startsWith("#")) {
            continue;
        }
        const eqIndex = trimmed.indexOf("=");
        if (eqIndex === -1) {
            continue;
        }
        const key = trimmed.slice(0, eqIndex).trim();
        const value = trimmed.slice(eqIndex + 1).trim();
        if (key !== "") {
            env[key] = value;
        }
    }

    return env;
}

export default defineConfig(() => {
    const env = parseEnvFile(resolve(import.meta.dirname, "server_ports.env"));
    const mapCallbackPort = env.MAP_CALLBACK_PORT ?? "8001";
    const assetServerPort = env.ASSET_SERVER_PORT ?? "8000";
    const viewerPort = Number.parseInt(env.MAP_VIEWER_PORT ?? "5173", 10);

    return {
        root: resolve(import.meta.dirname, "src/widgets/map_widget/frontend"),
        publicDir: resolve(import.meta.dirname, "app_data"),
        server: { host: "127.0.0.1", port: viewerPort, strictPort: true, hmr: false },
        define: {
            "import.meta.env.MAP_CALLBACK_PORT": JSON.stringify(mapCallbackPort),
            "import.meta.env.MAP_VIEWER_PORT": JSON.stringify(String(viewerPort)),
            "import.meta.env.ASSET_SERVER_PORT": JSON.stringify(assetServerPort)
        }
    };
});
