#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

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
}

GO_PORT=3002
VITE_PORT=5173

echo "Checking Go server port $GO_PORT..."
check_port "$GO_PORT"

echo "Checking Vite server port $VITE_PORT..."
check_port "$VITE_PORT"

os_type=$(uname -s | tr '[:upper:]' '[:lower:]')
arch_type=$(uname -m)
bin_name="server_${os_type}_${arch_type}"

if [[ "$os_type" == "linux"* ]]; then
    export QT_XCB_GL_INTEGRATION=none
    export XDG_SESSION_TYPE=x11
    export QT_QPA_PLATFORM=xcb
fi

go_src="src/widgets/map_widget/server.go"
python_src="src/main.py"
mkdir -p bin

command -v go >/dev/null || {
    echo "Go not installed."
    exit 1
}
command -v python3 >/dev/null || {
    echo "Python 3 not installed."
    exit 1
}
command -v bun >/dev/null || {
    echo "Bun not installed."
    exit 1
}

local_go=$(command -v go)
local_python=$(command -v python3)
local_bun=$(command -v bun)

should_rebuild=false
if command -v sha256sum >/dev/null; then
    src_hash=$(sha256sum "$go_src" | awk '{print $1}')
else
    src_hash=$(shasum -a 256 "$go_src" | awk '{print $1}')
fi

if [[ -f "bin/$bin_name" && -f last_build_time.txt ]]; then
    build_os_type=$(sed -n '1p' last_build_time.txt)
    build_arch_type=$(sed -n '2p' last_build_time.txt)
    last_build_time=$(sed -n '3p' last_build_time.txt)
    last_src_hash=$(sed -n '4p' last_build_time.txt || echo "")

    if ! [[ "$last_build_time" =~ ^[0-9]+$ ]]; then
        echo "Invalid build timestamp. Rebuilding..."
        should_rebuild=true
    else
        now=$(date +%s)
        if [[ "$build_os_type" != "$os_type" || "$build_arch_type" != "$arch_type" ]]; then
            echo "Binary built for $build_os_type/$build_arch_type, rebuilding..."
            should_rebuild=true
        elif ((now - last_build_time > 3600)); then
            echo "Binary is over 1 hour old. Rebuilding..."
            should_rebuild=true
        elif [[ "$last_src_hash" != "$src_hash" ]]; then
            echo "Source has changed. Rebuilding..."
            should_rebuild=true
        else
            echo "Binary is up to date. Skipping rebuild."
        fi
    fi
else
    echo "Binary missing or metadata missing. Rebuilding..."
    should_rebuild=true
fi

bun_packages_installed=false
if [[ -d "node_modules" && -f "bun.lock" ]]; then
    bun_packages_installed=true
fi

if [[ "$should_rebuild" == true ]]; then
    "$local_go" mod tidy
    "$local_go" build -o "bin/$bin_name" "$go_src"
    echo "Go server built successfully."
    {
        echo "$os_type"
        echo "$arch_type"
        date +%s
        echo "$src_hash"
    } >last_build_time.txt
fi

if [[ "$bun_packages_installed" == false ]]; then
    "$local_bun" install
fi

"$local_go" env >/dev/null

bin/$bin_name &
GO_PID=$!

"$local_bun" run serve &
VITE_PID=$!

"$local_python" "$python_src" &
PYTHON_PID=$!

cleanup() {
    [[ -n "${GO_PID:-}" ]] && kill "$GO_PID" 2>/dev/null || true
    [[ -n "${VITE_PID:-}" ]] && kill "$VITE_PID" 2>/dev/null || true
    [[ -n "${PYTHON_PID:-}" ]] && kill "$PYTHON_PID" 2>/dev/null || true

    [[ -n "${GO_PID:-}" ]] && wait "$GO_PID" 2>/dev/null || true
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
        kill -0 "$GO_PID" 2>/dev/null || break
        kill -0 "$VITE_PID" 2>/dev/null || break
        kill -0 "$PYTHON_PID" 2>/dev/null || break
        sleep 0.5
    done
fi

exit 0
