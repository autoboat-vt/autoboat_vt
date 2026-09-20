#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

# Source the shared server_ports.env so the server ports stay in sync with
# constants.py and vite.config.ts. server_ports.env is the single source of
# truth for all three local server ports (MAP_SERVER_PORT, VITE_PORT,
# ASSET_SERVER_PORT).
# shellcheck source=server_ports.env
if [[ -f server_ports.env ]]; then
    set -a
    # shellcheck disable=SC1091
    . ./server_ports.env
    set +a
fi

# Fallback defaults if server_ports.env is missing or doesn't set these.
: "${MAP_SERVER_PORT:=3002}"
: "${VITE_PORT:=5173}"
: "${ASSET_SERVER_PORT:=8000}"

query_port() {
    local port=$1
    if lsof -iTCP:"$port" -sTCP:LISTEN -t >/dev/null; then
        return 1
    else
        return 0
    fi
}

check_port() {
    local port=$1
    if ! query_port "$port"; then
        echo "Port $port is in use by the following process(es):"
        pids=$(lsof -iTCP:"$port" -sTCP:LISTEN -t)
        for pid in $pids; do
            user=$(ps -o user= -p "$pid")
            args=$(ps -o args= -p "$pid")
            start=$(ps -o lstart= -p "$pid")
            echo "PID: $pid | User: $user | Command: $args | Started: $start"
        done
        read -r -p "Do you want to kill these process(es)? [y/N] " answer
        case "$answer" in
        [Yy]*)
            lsof -iTCP:"$port" -sTCP:LISTEN -t | xargs kill -TERM >/dev/null 2>&1 || true
            for _ in {1..10}; do
                if query_port "$port"; then
                    break
                fi
                sleep 0.5
            done
            if ! query_port "$port"; then
                echo "Escalating to kill -KILL for remaining processes on port $port..."
                lsof -iTCP:"$port" -sTCP:LISTEN -t | xargs kill -KILL >/dev/null 2>&1 || true
                for _ in {1..10}; do
                    if query_port "$port"; then
                        break
                    fi
                    sleep 0.5
                done
            fi
            if ! query_port "$port"; then
                echo "Port $port still in use after kills. Exiting."
                exit 1
            fi
            ;;
        *)
            echo "Port $port is in use. Exiting."
            exit 1
            ;;
        esac
    fi
    echo "Port $port is free."
}

echo "Checking map server port $MAP_SERVER_PORT..."
check_port "$MAP_SERVER_PORT"

echo "Checking Vite server port $VITE_PORT..."
check_port "$VITE_PORT"

echo "Checking asset server port $ASSET_SERVER_PORT..."
check_port "$ASSET_SERVER_PORT"

os_type=$(uname -s | tr '[:upper:]' '[:lower:]')
if [[ "$os_type" == "linux"* ]]; then
    export QT_XCB_GL_INTEGRATION=none
    export XDG_SESSION_TYPE=x11
    export QT_QPA_PLATFORM=xcb
fi

if command -v python >/dev/null; then
    local_python=$(command -v python)
elif command -v python3 >/dev/null; then
    local_python=$(command -v python3)
else
    echo "Python is not installed."
    exit 1
fi

FRONTEND_DIST="src/widgets/map_widget/dist"
if [[ ! -f "$FRONTEND_DIST/index.html" ]]; then
    echo "Built frontend not found at $FRONTEND_DIST."
    if ! command -v bun >/dev/null; then
        echo "Bun is not installed, so the frontend cannot be built."
        echo "Install Bun (https://bun.sh) once to build the frontend, or use a packaged release."
        exit 1
    fi

    if [[ ! -d "node_modules" || ! -f "bun.lock" ]]; then
        echo "Installing frontend dependencies..."
        bun install
    fi

    echo "Building map frontend..."
    bun run build
fi

# The app now serves the built frontend itself on $VITE_PORT — no Vite
# dev server needed at runtime. For frontend development with live reload,
# run `bun run serve` in a separate terminal and set GROUND_STATION_HOME to
# this directory first.
"$local_python" "src/main.py" &
PYTHON_PID=$!

cleanup() {
    [[ -n "${PYTHON_PID:-}" ]] && kill "$PYTHON_PID" 2>/dev/null || true
    [[ -n "${PYTHON_PID:-}" ]] && wait "$PYTHON_PID" 2>/dev/null || true

    temp_file="app_data/git_ignore/app_state.json"
    [[ -f "$temp_file" ]] && rm "$temp_file"
}

trap 'cleanup' EXIT TERM INT

wait "$PYTHON_PID" 2>/dev/null || true

exit 0
