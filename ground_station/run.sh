#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

# source server_ports.env if it exists, to set the ports for the servers
if [[ -f server_ports.env ]]; then
    set -a
    . ./server_ports.env
    set +a
fi

# fallback defaults if server_ports.env is missing or doesn't set these
: "${MAP_CALLBACK_PORT:=8001}"
: "${MAP_VIEWER_PORT:=5173}"
: "${ASSET_SERVER_PORT:=8000}"

os_type=$(uname -s | tr '[:upper:]' '[:lower:]')
if [[ "$os_type" == "linux"* ]]; then
    export QT_XCB_GL_INTEGRATION=none
    export QT_OPENGL=software
    export QT_QUICK_BACKEND=software
    export QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:-} --disable-gpu --disable-gpu-compositing"
    export XDG_SESSION_TYPE=x11
    export QT_QPA_PLATFORM=xcb

    if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
        export XDG_RUNTIME_DIR="/tmp/runtime-${USER:-$(id -un)}"
        mkdir -p "$XDG_RUNTIME_DIR"
        chmod 700 "$XDG_RUNTIME_DIR"
    fi
fi

if command -v python >/dev/null; then
    local_python=$(command -v python)
elif command -v python3 >/dev/null; then
    local_python=$(command -v python3)
else
    echo "Python is not installed."
    exit 1
fi

"$local_python" "src/main.py" &
PYTHON_PID=$!

cleanup() {
    [[ -n "${PYTHON_PID:-}" ]] && kill "$PYTHON_PID" 2>/dev/null || true
    [[ -n "${PYTHON_PID:-}" ]] && wait "$PYTHON_PID" 2>/dev/null || true

    temp_file="app_data/git_ignore/app_state.json"
    [[ -f "$temp_file" ]] && rm "$temp_file"
}

trap 'cleanup' EXIT TERM INT

if wait -n 2>/dev/null; then
    :
else
    while true; do
        kill -0 "$PYTHON_PID" 2>/dev/null || break
        sleep 0.5
    done
fi

exit 0
