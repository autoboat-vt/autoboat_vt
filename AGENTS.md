# AutoBoat VT — Agent Guidelines

This repository is the main software package for the **Virginia Tech AutoBoat** autonomous surface vessel team. It contains a ROS 2 (Humble) workspace, a PyQt ground station with a Vite/TypeScript map frontend, Raspberry Pi Pico firmware (C/C++ + micro-ROS), Gazebo simulation plugins, and Docker/devcontainer tooling.

Public docs: <https://autoboat-vt.github.io/documentation>

## Repository Layout

| Path | Stack | Purpose |
|------|-------|---------|
| `ros_packages/` | Python + C++ (ROS 2 Humble) | All colcon packages: `autoboat_msgs`, `autopilot`, `autopilot_cpp`, `drivers`, `drivers_cpp`, `object_detection`, `simulation/*`, `old_sailboat_simulation`, `autoboat_launch` |
| `ground_station/` | Python (qtpy/PySide) + TypeScript (Vite, Leaflet) | Desktop GUI; embeds a `QWebEngineView` running a vanilla-TS map widget |
| `firmware/` | C/C++ (Pico SDK + micro-ROS) | RP2040 firmware running on the boat |
| `.devcontainer/` | Docker | Dev container variants: base, jetson, deepstream, firmware_dependencies |
| `scripts/` | Bash | Docker image build/push, Jetson install helpers |
| `install.sh` | Bash | End-user apt install of prebuilt `.deb` packages |
| `biome.jsonc` | — | Linter/formatter for TS/JS/CSS/HTML/JSON (root config) |
| `ruff.toml` | — | Linter/formatter for all Python |
| `taplo.toml` | — | Formatter for TOML |

The top-level `build/`, `install/`, `log/` directories (and per-package `__pycache__/`, `*.egg-info/`, `CMakeCache.txt`, etc.) are colcon build artifacts — do not edit them. Source of truth is `ros_packages/`.

## Build & Run

The devcontainer is the canonical environment (ROS Humble, `/home/ws` workspace mount). Inside it:

```bash
# Full build (Python + C++)
build
# Python-only (skips *_cpp packages) — faster during ground_station/ros Python work
build_python
```

These are aliases defined in `.devcontainer/postCreateCommand.sh` that expand to:
```bash
cd /home/ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

`--symlink-install` means Python edits take effect without rebuild. C++ changes always require a rebuild.

Ground station (run on a host with display, not in the headless container):
```bash
cd ground_station && ./run.sh
```
This starts the Vite map server on `127.0.0.1:5173` and launches the PyQt app.

CI release build: `.github/workflows/build_ros_packages.sh` (expects `DEB_VERSION`, `DEB_ARCH`, `IGNORE_PACKAGES` env vars; uses `mold` linker).

## Conventions

- **Python**: ruff with `select = ["ALL"]` and a curated ignore list (see `ruff.toml`). Strict typing, `from __future__ import annotations`, numpy-style docstrings with `Parameters`/`Returns`/`Inherits`/`Notes`/`Raises` sections, `__all__` in modules. Line length 130, 4-space indent.
- **TypeScript/JS/CSS/HTML/JSON**: Biome (root `biome.jsonc`). Double quotes, semicolons, 4-space indent (HTML/CSS 4-space), 130-char lines (HTML/CSS 100). `noConsole: off`, `noDebugger: error`, `noUnusedImports: error`.
- **TOML**: taplo with `align_entries = true`, 4-space indent.
- **C++**: C++23 for `autopilot_cpp`, C++17 for `firmware` (Pico SDK). `-Wall -Wextra -Wpedantic`. Use `ccache` and `mold` when available (auto-detected by CMake).
- **ROS 2**: ament_python for pure-Python packages, ament_cmake for C++ packages. All ROS nodes publish/subscribe on `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` inside the devcontainer.
- **Enums shared across repos**: `ground_station/src/utils/constants.py::StrictMatchEnums` MUST stay in sync with `ros_packages/autopilot/autopilot/autopilot_library/utils/constants.py`. Changing one requires changing the other.
- **Ground station state**: persistent config lives in `ground_station/src/utils/state_manager.py::StateManager` (singleton `constants.SM`), backed by `app_data/git_ignore/app_state.json`. Add new defaults to `constants.STATE_FILE_CONTENTS` before reading.
- **No commit secrets**: `app_data/git_ignore/` is gitignored by name — keep runtime state there.
- **Writing style**: do not use emojis, decorative unicode, or characters/tics that only an LLM would produce (e.g. `✨`, `🚀`, `👉`, `—` em dashes where a hyphen suffices, curly quotes `“”` instead of straight `"`, heavy use of `**bold**` for emphasis). Write in plain ASCII. The only exception is the `⚠️` marker, which is used deliberately in `.github/instructions/` files and `AGENTS.md` to flag critical gotchas/maintenance rules — do not sprinkle it elsewhere. Code, comments, commit messages, PR descriptions, and documentation should read like they were written by a human teammate, not an AI.

## Instruction Files

Stack-specific guidance lives in `.github/instructions/*.instructions.md`. Each file has YAML frontmatter (`description` + optional `applyTo` glob) and auto-attaches when a matching file is in context, or loads on-demand when the agent detects task relevance from the `description` keywords.

> ### ⚠️ CRITICAL: Keep these files and `AGENTS.md` up to date
>
> These files are **living documents**, not write-once docs. They MUST be updated whenever the codebase changes. **Stale instructions are worse than no instructions** — they cause the agent to confidently apply outdated patterns, reference removed APIs, or enforce dropped conventions.
>
> **Before finishing any task that touches the codebase, ask: "Did this change invalidate anything in `.github/instructions/` or `AGENTS.md`?" If yes, update the affected file(s) in the same PR.** Do not defer instruction updates to a separate PR — they will be forgotten.
>
> See `.github/instructions/creating-instructions.instructions.md` → "⚠️ CRITICAL: Keep these files up to date" for the full maintenance trigger list and maintenance checklist.

| File | `applyTo` | Domain |
|------|-----------|--------|
| `python.instructions.md` | `**/*.py` | ruff, ROS 2 Python nodes, StateManager, telemetry |
| `typescript.instructions.md` | `**/*.ts, **/*.js, **/*.css, **/*.html, **/*.jsonc, **/biome.json, **/biome.jsonc` | Biome, Vite/Leaflet map widget, MapInterface, MarkerManager |
| `cpp.instructions.md` | `**/*.cpp, **/*.hpp, **/*.h, **/*.cc, **/CMakeLists.txt` | C++23/C++17, CMakeLists, autopilot_cpp/drivers_cpp |
| `ros-packages.instructions.md` | `ros_packages/**` | ament_python/cmake, autoboat_msgs, launch files, Gazebo plugins |
| `ground-station.instructions.md` | `ground_station/**` | PyQt, StateManager, AutopilotConfigWidget, map_widget server |
| `firmware.instructions.md` | `firmware/**` | RP2040 Pico SDK, micro-ROS, per-peripheral struct pattern |
| `devcontainer-scripts.instructions.md` | `.devcontainer/**, scripts/**, install.sh, .github/workflows/**` | Docker, DeepStream, Pico SDK install, CI |
| `creating-instructions.instructions.md` | `.github/instructions/**, **/*.instructions.md, .github/copilot-instructions.md` | How to create/edit instruction files themselves |

## When Making Changes

- Read the relevant `.github/instructions/*.instructions.md` file — they auto-attach by file pattern and contain stack-specific gotchas.
- After Python edits, run `ruff check --fix` and `ruff format` on the changed files. After TS/JS/CSS edits, run `biome check --write` from `ground_station/` (Biome config discovery walks up; pass `--config-path` if a parent dir has its own `biome.json`).
- For new ROS messages in `ros_packages/autoboat_msgs/msg/`, expect a colcon rebuild and downstream C++/Python updates.
- The `old_sailboat_simulation/` package is deprecated — prefer `ros_packages/simulation/` for new simulation work.
- When creating a new instruction file, follow `.github/instructions/creating-instructions.instructions.md` — it documents the frontmatter schema, the critical `applyTo` string-not-array gotcha, the "Use when..." description pattern, and the full creation checklist. Add the new file to the table above.
- **⚠️ Update the affected `.github/instructions/*.instructions.md` file(s) and `AGENTS.md` whenever your change alters a package, node, topic, message, class, function, widget, build command, devcontainer variant, linter config, or documented gotcha.** This is mandatory, not optional. Stale instructions mislead every subsequent agent session and teammate. Commit the instruction updates **in the same PR** as the code change — never defer to a separate PR. Verify every code block, class name, and path against current source via `read_file` before editing the instruction file.

## Things To Avoid

- Using emojis, decorative unicode (curly quotes, em dashes, arrows like `→` outside of code), or LLM-typical tics (`✨`, `🚀`, `👉`, excessive `**bold**`) in code, comments, commit messages, PR descriptions, or docs. Write plain ASCII like a human teammate would. The only sanctioned exception is the `⚠️ marker used in `.github/instructions/` files and `AGENTS.md` to flag critical gotchas.
- Editing files under `build/`, `install/`, `log/`, or any `__pycache__/` / `*.egg-info/` — these are build outputs.
- **⚠️ Leaving `.github/instructions/*.instructions.md` or `AGENTS.md` stale after a codebase change** — stale instructions cause the agent to apply outdated patterns and reference removed APIs. Update them in the same PR as the code change, always. Trust source over memory — `read_file` the actual code before editing an instruction file.
- Using `applyTo: ["a", "b"]` array form in any `.instructions.md` frontmatter — VS Code Copilot rejects it; use comma-separated string form (`applyTo: "a, b"`).
- Adding PyQt/PySide code that imports `PyQt5` or `PyQt6` directly — always go through `qtpy` for Qt abstraction.
- Adding new TS files for the map widget outside `ground_station/src/widgets/map_widget/frontend/` — `tsconfig.json` `include` is hard-scoped to that directory.
