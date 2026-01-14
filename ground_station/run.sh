#!/usr/bin/env -S bash -euo pipefail

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
        for pid in $(lsof -iTCP:"$port" -sTCP:LISTEN -t); do
            user=$(ps -o user= -p "$pid")
            cmd=$(ps -o comm= -p "$pid")
            start=$(ps -o lstart= -p "$pid")
            echo "PID: $pid | User: $user | Command: $cmd | Started: $start"
        done

        read -p "Do you want to kill these process(es)? [y/N] " answer
        case "$answer" in
        [Yy]*)
            lsof -iTCP:"$port" -sTCP:LISTEN -t | xargs kill -9 >/dev/null 2>&1 || true
            # wait until port is actually free
            for i in {1..10}; do
                if query_port "$port"; then
                    break
                fi
                sleep 0.5
            done
            if ! query_port "$port"; then
                echo "Port $port still in use after kill. Exiting."
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

GO_PORT=3001
ASSET_SERVER_PORT=8000
CDN_SERVER_PORT=8080

echo "Checking Go server port $GO_PORT..."
check_port "$GO_PORT"

echo "Checking Asset server port $ASSET_SERVER_PORT..."
check_port "$ASSET_SERVER_PORT"

echo "Checking CDN server port $CDN_SERVER_PORT..."
check_port "$CDN_SERVER_PORT"

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

local_go=$(command -v go)
local_python=$(command -v python3)

should_rebuild=false
src_hash=$(sha256sum "$go_src" | awk '{print $1}')

if [[ -f "bin/$bin_name" && -f last_build_time.txt ]]; then
    build_os_type=$(sed -n '1p' last_build_time.txt)
    build_arch_type=$(sed -n '2p' last_build_time.txt)
    last_build_time=$(sed -n '3p' last_build_time.txt)
    last_src_hash=$(sed -n '4p' last_build_time.txt || echo "")

    now=$(date +%s)

    if [[ "$build_os_type" != "$os_type" || "$build_arch_type" != "$arch_type" ]]; then
        echo "Binary built for $build_os_type/$build_arch_type; rebuilding..."
        should_rebuild=true
    elif ((now - last_build_time > 3600)); then
        echo "Binary is over 1 hour old. Rebuilding..."
        should_rebuild=true
    elif [[ "$last_src_hash" != "$src_hash" ]]; then
        echo "Source has changed. Rebuilding..."
        should_rebuild=true
    else
        echo "Binary is up-to-date. Skipping rebuild."
    fi
else
    echo "Binary missing or metadata missing. Rebuilding..."
    should_rebuild=true
fi

if [[ "$should_rebuild" == true ]]; then
    $local_go mod tidy
    $local_go build -o "bin/$bin_name" "$go_src"
    echo "Go server built successfully."
    {
        echo "$os_type"
        echo "$arch_type"
        date +%s
        echo "$src_hash"
    } >last_build_time.txt
fi

bin/$bin_name &
GO_PID=$!

$local_python "$python_src" &
PYTHON_PID=$!

cleanup() {
    kill "$GO_PID" "$PYTHON_PID" 2>/dev/null || true
    wait "$GO_PID" "$PYTHON_PID" 2>/dev/null || true
    temp_file="src/widgets/autopilot_param_editor/params_temp.json"
    [[ -f $temp_file ]] && rm "$temp_file"
}

trap cleanup EXIT TERM INT
wait -n

exit 0
