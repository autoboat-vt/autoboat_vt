#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GS_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
cd "$GS_DIR"

SKIP_FRONTEND=false
SKIP_PYINSTALLER=false
for arg in "$@"; do
    case "$arg" in
    --skip-frontend) SKIP_FRONTEND=true ;;
    --skip-pyinstaller) SKIP_PYINSTALLER=true ;;
    *)
        echo "Unknown flag: $arg" >&2
        exit 2
        ;;
    esac
done

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
    if [[ -d ".venv" ]]; then
        if .venv/bin/python -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
            PYTHON_BIN=".venv/bin/python"
            [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]] && PYTHON_BIN=".venv/Scripts/python.exe"
        else
            echo "Existing .venv uses Python < 3.10; recreating it (venv is disposable) ..."
            rm -rf .venv
        fi
    fi
    if [[ -z "$PYTHON_BIN" ]]; then
        for candidate in python3.13 python3.12 python3.11 python3.10 python3 python; do
            if command -v "$candidate" >/dev/null 2>&1; then
                if "$candidate" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
                    PYTHON_BIN=$(command -v "$candidate")
                    break
                fi
            fi
        done
    fi
fi
if [[ -z "$PYTHON_BIN" ]]; then
    echo "Python >= 3.10 is required to build the Ground Station." >&2
    echo "Install it (https://www.python.org / brew / pyenv) or set PYTHON_BIN." >&2
    exit 1
fi
echo "Using Python: $PYTHON_BIN ($($PYTHON_BIN --version 2>&1))"

if [[ ! -d ".venv" ]]; then
    echo "Creating virtual environment in .venv ..."
    "$PYTHON_BIN" -m venv .venv
    PYTHON_BIN=".venv/bin/python"
    [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]] && PYTHON_BIN=".venv/Scripts/python.exe"
fi

echo "Installing Python dependencies ..."
"$PYTHON_BIN" -m pip install --upgrade pip >/dev/null
PYTHONUTF8=1 "$PYTHON_BIN" -m pip install -r "$GS_DIR/../.devcontainer/groundstation_required_pip_packages.txt"

if [[ "$SKIP_FRONTEND" == false ]]; then
    FRONTEND_DIST="src/widgets/map_widget/dist"
    FRONTEND_SRC_DIR="src/widgets/map_widget/frontend"

    # Rebuild if dist/ is missing OR any TS/HTML source is newer than dist/index.html.
    # `find ... -newer ... -print -quit` prints nothing when nothing is newer, so
    # an empty result means dist is up-to-date.
    NEWER_THAN_DIST=$(
        find "$FRONTEND_SRC_DIR" \
            -type f \( -name '*.ts' -o -name '*.tsx' -o -name '*.html' -o -name '*.css' \) \
            -newer "$FRONTEND_DIST/index.html" \
            -print -quit 2>/dev/null
    )

    if [[ ! -f "$FRONTEND_DIST/index.html" || -n "$NEWER_THAN_DIST" ]]; then
        if command -v bun >/dev/null; then
            echo "Building map frontend with bun ..."
            if [[ ! -d "node_modules" ]]; then
                bun install
            fi
            bun run build
        else
            echo "Built frontend not found or stale, and bun is not installed." >&2
            echo "Install bun (https://bun.sh) or download a prebuilt release." >&2
            exit 1
        fi
    else
        echo "Map frontend already built (skipping)."
    fi
fi

if [[ "$SKIP_PYINSTALLER" == false ]]; then
    echo "Running PyInstaller ..."

    rm -rf dist/ground_station dist/GroundStation.app packaging/build
    "$PYTHON_BIN" -m PyInstaller --clean --noconfirm \
        --workpath packaging/build \
        --distpath dist \
        packaging/ground_station.spec

    if [[ "$OSTYPE" == "darwin"* ]]; then
        RELEASE_SRC="dist/GroundStation.app"

        echo "Re-signing macOS bundle (ad-hoc, deep) ..."
        find "dist/GroundStation.app" -name '_CodeSignature' -prune -o -print0 |
            xargs -0 xattr -c 2>/dev/null || true
        codesign --deep --force --sign - --options runtime \
            --entitlements /dev/null \
            "dist/GroundStation.app" 2>&1 ||
            codesign --deep --force --sign - "dist/GroundStation.app"

        # Verify the seal so the failure surfaces during build, not later.
        if ! spctl -a -v --type execute "dist/GroundStation.app" 2>&1; then
            echo "WARNING: Gatekeeper still rejects the signed bundle." >&2
        fi

        echo "macOS bundle at $RELEASE_SRC"
    else
        RELEASE_SRC="dist/ground_station"
    fi

    mkdir -p "dist/ground_station/app_data/git_ignore"
    echo
    echo "=============================================================="
    echo " Build complete."
    echo " App:     $RELEASE_SRC"
    echo " Data:    dist/ground_station/app_data/  (writable, travels with the app)"
    echo "=============================================================="
fi
