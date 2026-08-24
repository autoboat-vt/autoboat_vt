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

command -v python3 >/dev/null || {
    echo "Python 3 not installed."
    exit 1
}
command -v bun >/dev/null || {
    echo "Bun not installed."
    exit 1
}

local_python=$(command -v python3)
local_bun=$(command -v bun)

bun_packages_installed=false
if [[ -d "node_modules" && -f "bun.lock" ]]; then
    bun_packages_installed=true
fi

if [[ "$bun_packages_installed" == false ]]; then
    "$local_bun" install
fi

"$local_bun" run serve &
VITE_PID=$!

"$local_python" "src/main.py" &
PYTHON_PID=$!

cleanup() {
    [[ -n "${VITE_PID:-}" ]] && kill "$VITE_PID" 2>/dev/null || true
    [[ -n "${PYTHON_PID:-}" ]] && kill "$PYTHON_PID" 2>/dev/null || true

    [[ -n "${VITE_PID:-}" ]] && wait "$VITE_PID" 2>/dev/null || true
    [[ -n "${PYTHON_PID:-}" ]] && wait "$PYTHON_PID" 2>/dev/null || true

    temp_file="app_data/git_ignore/app_state.json"
    [[ -f "$temp_file" ]] && rm "$temp_file"
}

trap 'cleanup' EXIT TERM INT

if wait -n 2>/dev/null; then
    :
else
    while true; do
        kill -0 "$VITE_PID" 2>/dev/null || break
        kill -0 "$PYTHON_PID" 2>/dev/null || break
        sleep 0.5
    done
fi

exit 0
