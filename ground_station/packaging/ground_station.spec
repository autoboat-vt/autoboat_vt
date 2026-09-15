"""
PyInstaller spec for the Autoboat Ground Station native app.

Produces an **onedir** bundle named ``ground_station`` (one folder plus a
launcher executable) so that:

- the folder can sit next to user data under the same parent directory;
- macOS/Linux gatekeeper/quarantine steps stay simple (zip the folder);
- startup is a normal double-click (or ``./ground_station_app``).

Data files layout inside the bundle::

    _internal/
        ground_station/
            src/                      # collected Python source
            app_data/git_keep/        # assets + defaults (read-only payload)
            src/widgets/map_widget/dist/  # built map frontend

Runtime-writable state lives OUTSIDE the bundle, in the folder containing
the executable (``app_data/git_ignore/...``) — see ``src/main.py``.

Build::

    pyinstaller packaging/ground_station.spec --clean --noconfirm
"""

from __future__ import annotations

import sys
from pathlib import Path

from PyInstaller.building.datastruct import TOC
from PyInstaller.utils.hooks import collect_data_files

block_cipher = None

_PACKAGING_DIR = Path(SPECPATH).resolve()  # ground_station/packaging/
GS_DIR = _PACKAGING_DIR.parent  # ground_station/
SRC_DIR = GS_DIR / "src"

_DATAS: list[tuple[str, str]] = []

# Built map frontend (must exist — run `bun run build` first; the wrapper
# build_app.sh does this automatically).
_frontend_dist = SRC_DIR / "widgets" / "map_widget" / "dist"
if not (_frontend_dist / "index.html").is_file():
    raise SystemExit(
        f"Built frontend not found at {_frontend_dist}. "
        "Run `bun install && bun run build` in ground_station/ first."
    )
_DATAS.append((str(_frontend_dist), "ground_station/src/widgets/map_widget/dist"))

# Read-only payload: assets + defaults next to the app.
_APP_DATA = GS_DIR / "app_data" / "git_keep"
_DATAS.append((str(_APP_DATA / "assets"), "ground_station/app_data/git_keep/assets"))
_DATAS.append((str(_APP_DATA / "defaults_examples"), "ground_station/app_data/git_keep/defaults_examples"))

# Port configuration is baked in as defaults, but ship the file so power
# users can see the layout reference.
_PORTS_ENV = GS_DIR / "server_ports.env"
if _PORTS_ENV.is_file():
    _DATAS.append((str(_PORTS_ENV), "ground_station"))

# OS-specific executable bits. On Linux the raw onedir output is used as-is;
# on macOS a .app bundle is produced below; Windows adds an icon + version
# metadata through the manifest resource only (kept minimal).
_IS_MAC = sys.platform == "darwin"
_IS_WIN = sys.platform == "win32"

_icon: str | None = None
if _IS_MAC:
    _icon_file = _PACKAGING_DIR / "icons" / "app.icns"
    _icon = str(_icon_file) if _icon_file.is_file() else None
elif _IS_WIN:
    _icon_file = _PACKAGING_DIR / "icons" / "app.ico"
    _icon = str(_icon_file) if _icon_file.is_file() else None


a = Analysis(
    [str(SRC_DIR / "main.py")],
    pathex=[str(SRC_DIR)],
    binaries=[],
    datas=_DATAS,
    hiddenimports=[
        # qtpy lazy-imports the binding pieces at runtime
        "PySide6.QtWebEngineWidgets",
        "PySide6.QtWebEngineCore",
        "PySide6.QtWebChannel",
        "PySide6.QtNetwork",
        "PySide6.QtPrintSupport",
        "pyqtgraph",
        "qtawesome",
        "pyqtgraph.graphicsItems.ViewBox",
        "pyqtgraph.graphicsItems.PlotItem",
        "pyqtgraph.graphicsItems.AxisItem",
        "pyqtgraph.imageview",
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # Other Qt bindings must not be dragged in (they fight over QT_API)
        "PyQt5",
        "PyQt6",
        "PySide2",
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

# ---------------------------------------------------------------------------
# Linux: strip bundled copies of core runtime libraries that MUST come from
# the host system, not the bundle.
#
# PyInstaller bundles the build machine's libstdc++.so.6 into _internal/.
# When the process loads a system-shared library that itself needs a NEWER
# libstdc++ (e.g. Intel VA-API: iHD_drv_video.so -> libigdgmm.so.12 needs
# GLIBCXX_3.4.32), the dynamic linker resolves those symbols against the
# already-loaded, OLDER bundled libstdc++ and aborts. These libraries are
# backward-compatible, so the system copy (>= bundled) always satisfies the
# app; the reverse is not true. Drop them from the bundle.
# ---------------------------------------------------------------------------
if not _IS_MAC and not _IS_WIN:
    _STRIP_LIBS = {
        "libstdc++.so.6",  # see block comment above
        "libgcc_s.so.1",   # pairs with libstdc++; same reasoning
    }
    a.binaries = TOC(
        [entry for entry in a.binaries if Path(entry[0]).name not in _STRIP_LIBS]
    )

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="ground_station_app",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=not _IS_WIN,  # keep the console on mac/linux for logs; hidden on Windows
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=_icon,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="ground_station",
)

if _IS_MAC:
    app = BUNDLE(
        coll,
        name="GroundStation.app",
        icon=_icon,
        bundle_identifier="edu.vt.autoboat.groundstation",
    )
