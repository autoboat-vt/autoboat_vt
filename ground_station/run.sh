#!/usr/bin/env -S bash -euo pipefail

os_type=$(uname -s | tr '[:upper:]' '[:lower:]')

if [[ "$os_type" == "linux"* ]]; then
    export QT_XCB_GL_INTEGRATION=none
    export XDG_SESSION_TYPE=x11
    export QT_QPA_PLATFORM=xcb
fi

# Check for Go installation
if ! command -v go &> /dev/null; then
    echo "Go is not installed. Please install Go to run this script."
    exit 1
fi

go_src="src/widgets/map_widget/server.go"
python_src="src/main.py"

# Check for Python installation
if ! command -v python3 &> /dev/null; then
    echo "Python 3 is not installed. Please install Python 3 to run this script."
    exit 1
fi

local_go=$(command -v go)
local_python=$(command -v python3)

mkdir -p bin
bin_name="server_$os_type"

should_rebuild=false

if [[ -f "bin/$bin_name" ]]; then
    if [[ -f "last_build_time.txt" ]]; then
        build_os_type=$(sed -n '1p' last_build_time.txt)
        if [[ "$build_os_type" != "$os_type" ]]; then
            echo "Server binary was built for $build_os_type, but this is $os_type. Rebuilding..."
            should_rebuild=true
        fi

        last_build_time=$(sed -n '2p' last_build_time.txt)
        now=$(date +%s)
        if (( now - last_build_time > 3600 )); then
            echo "Server binary is over 1 hour old. Rebuilding..."
            should_rebuild=true
        else
            echo "Server binary is fresh. Skipping rebuild."
        fi
    else
        echo "No last_build_time.txt file found. Rebuilding..."
        should_rebuild=true
    fi
else
    echo "Server binary not found. Building..."
    should_rebuild=true
fi

if [[ "$should_rebuild" == true ]]; then
    $local_go mod tidy
    if $local_go build -o "bin/$bin_name" $go_src; then
        echo "Go server built successfully."
        {
            echo "$os_type"
            date +%s
        } > last_build_time.txt
    else
        echo "Failed to build the Go server."
        exit 1
    fi
fi

# Start the Go server in the background
bin/$bin_name &
GO_PID=$!

# Start Python script in the background
$local_python $python_src &
PYTHON_PID=$!

cleanup() {
    kill "$GO_PID" "$PYTHON_PID" 2>/dev/null || true
    wait "$GO_PID" "$PYTHON_PID" 2>/dev/null || true

    if [[ -f "src/widgets/autopilot_param_editor/params_temp.json" ]]; then
        rm "src/widgets/autopilot_param_editor/params_temp.json"
    fi
}
trap cleanup EXIT

# Wait for either process to exit, then trigger cleanup
wait -n "$GO_PID" "$PYTHON_PID"
exit 0
