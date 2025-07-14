#!/usr/bin/env bash

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    export QT_XCB_GL_INTEGRATION=none
    export XDG_SESSION_TYPE=x11
    export QT_QPA_PLATFORM=xcb
fi

# Check for Go installation
if ! command -v go &> /dev/null; then
    echo "Go is not installed. Please install Go to run this script."
    exit 1
fi

# Check for Python installation
if ! command -v python3 &> /dev/null; then
    echo "Python 3 is not installed. Please install Python 3 to run this script."
    exit 1
fi

local_go=$(which go)
local_python=$(which python3)

mkdir -p bin

if [ -f "bin/server" ]; then
    last_build_time=$(cat "last_build_time.txt")
    # if last_build_time is older than 1 hour
    if [ $(($(date +%s) - last_build_time)) -gt 3600 ]; then
        echo "Server binary out of date. Rebuilding..."
        $local_go mod tidy
        $local_go build -o bin/server src/web_engine/server.go
        if [ $? -ne 0 ]; then
            echo "Failed to build the Go server."
            exit 1
        else
            echo "Go server built successfully."
            date +%s > last_build_time.txt
        fi
    else
        echo "last_build_time.txt is less than 1 hour old. Skipping build."
    fi
else
    echo "Server binary not found. Building..."
    $local_go mod tidy
    $local_go build -o bin/server src/web_engine/server.go
    if [ $? -ne 0 ]; then
        echo "Failed to build the Go server."
        exit 1
    else
        echo "Go server built successfully."
        date +%s > last_build_time.txt
    fi
fi

# Start the Go server in the background
bin/server & 
GO_PID=$!

# Start Python script in the background
$local_python src/main.py &
PYTHON_PID=$!

# Trap Ctrl+C and kill both processes
trap "kill $GO_PID $PYTHON_PID" SIGINT

# Wait for both processes to finish
wait $GO_PID
wait $PYTHON_PID

# Cleanup
if [ -f "src/widgets/autopilot_param_editor/params_temp.json" ]; then
    rm src/widgets/autopilot_param_editor/params_temp.json
fi
