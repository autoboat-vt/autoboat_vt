---
description: "Use when creating, editing, or reviewing .instructions.md files, AGENTS.md, or copilot-instructions.md. Covers YAML frontmatter schema (description required, applyTo optional), the critical applyTo string-not-array gotcha (VS Code Copilot validator rejects YAML arrays), discovery modes (on-demand via description, explicit via applyTo, manual), the 'Use when...' description pattern, one-concern-per-file rule, show-don't-tell code examples, the 7 existing instruction files in this repo, and the full file creation checklist."
applyTo: ".github/instructions/**, **/*.instructions.md, .github/copilot-instructions.md"
---

# Creating Instruction Files

Instruction files (`.instructions.md`) are the primary mechanism for stack-specific and file-pattern-specific guidance in this repo. There are currently **7** under `.github/instructions/`.

## ⚠️ CRITICAL: Keep these files up to date

**The instruction files and `AGENTS.md` are living documents. They MUST be updated whenever the codebase changes.** Stale instructions are worse than no instructions — they cause the agent to confidently apply outdated patterns, reference removed APIs, or enforce dropped conventions.

**Update the relevant `.instructions.md` file AND `AGENTS.md` whenever you:**

- Add, remove, or rename a ROS package, node, topic, message type, launch file, or config file.
- Add, remove, or rename a Python module, class, function, or constant (especially `StateManager` keys, `StrictMatchEnums`, topic names).
- Add, remove, or rename a C++ class, library, CMake target, or `package.xml` dependency.
- Add, remove, or rename a ground station widget, dialog, or map widget frontend file.
- Add, remove, or rename a firmware peripheral, device class, or `BOAT_TYPE`.
- Change a build command, devcontainer variant, Docker image name, or CI workflow.
- Change a linter/formatter config (`ruff.toml`, `biome.jsonc`, `taplo.toml`) or a convention (indent, line length, import order).
- Discover a new gotcha, drift, or bug in the codebase (add it as a ⚠️ note).
- Add or remove an instruction file itself (update the table in `AGENTS.md` → "Instruction Files").

**Before finishing any task that touches the codebase, ask: "Did this change invalidate anything in `.github/instructions/` or `AGENTS.md`?" If yes, update the affected file(s) in the same PR.** Do not defer instruction updates to a separate PR — they will be forgotten.

**When updating an instruction file:**

1. Verify every code block, class name, function signature, file path, and topic name against the current source (don't trust memory — `read_file` the actual source).
2. If a documented gotcha has been fixed in code, remove the ⚠️ note (don't leave stale warnings).
3. If a new gotcha was discovered, add it with a ⚠️ marker.
4. Update the `description` frontmatter if new keywords are needed for discovery.
5. Update the table in `AGENTS.md` → "Instruction Files" if the file's `applyTo` or domain summary changed.
6. Run `get_errors` on the edited file to confirm no validation issues.

**These files are checked into git and shared across the team.** A stale instruction file misleads every teammate and every agent session that touches that part of the codebase. Treat them as part of the code, not as documentation-after-the-fact.

| File | `applyTo` | Domain |
|------|-----------|--------|
| `python.instructions.md` | `**/*.py` | ruff, ROS 2 Python nodes, StateManager, telemetry |
| `typescript.instructions.md` | `**/*.ts, **/*.js, **/*.css, **/*.html, **/*.jsonc, **/biome.json, **/biome.jsonc` | Biome, Vite/Leaflet map widget, MapInterface, MarkerManager |
| `cpp.instructions.md` | `**/*.cpp, **/*.hpp, **/*.h, **/*.cc, **/CMakeLists.txt` | C++23/C++17, CMakeLists, autopilot_cpp/drivers_cpp |
| `ros-packages.instructions.md` | `ros_packages/**` | ament_python/cmake, autoboat_msgs, launch files, Gazebo plugins |
| `ground-station.instructions.md` | `ground_station/**` | PyQt, StateManager, AutopilotConfigWidget, map_widget server |
| `firmware.instructions.md` | `firmware/**` | RP2040 Pico SDK, micro-ROS, per-peripheral struct pattern |
| `devcontainer-scripts.instructions.md` | `.devcontainer/**, scripts/**, install.sh, .github/workflows/**` | Docker, DeepStream, Pico SDK install, CI |

## File location

```
.github/instructions/<name>.instructions.md
```

Workspace-scoped (team-shared). User-profile-scoped instructions live in `{{VSCODE_USER_PROMPTS_FOLDER}}/instructions/` — do NOT use those for repo-shared guidance.

## Frontmatter schema

```yaml
---
description: "<required — keyword-rich, 'Use when...' pattern>"
applyTo: "<optional — comma-separated glob patterns>"
name: "Optional Display Name"
---
```

- `description` is **required** — this is the discovery surface. If trigger phrases aren't in the description, the agent won't find the file for on-demand loading.
- `applyTo` is **optional** — when present, the file auto-attaches whenever a file matching the glob is in context (being created or edited).
- `name` is optional — defaults to the filename.

### ⚠️ CRITICAL: `applyTo` must be a STRING, not a YAML array

The VS Code Copilot validator **rejects** the YAML array form despite some reference docs showing it as valid:

```yaml
# ❌ WRONG — validator error: "The 'applyTo' attribute must be a string."
applyTo: ["src/**", "lib/**"]

# ✅ RIGHT — comma-separated string
applyTo: "src/**, lib/**"

# ✅ RIGHT — single pattern string
applyTo: "**/*.py"
```

Symptom: the instruction file silently fails to auto-attach. The error surfaces in the customizations panel, not in the file itself.

**Always use the comma-separated string form** for multiple patterns. This is also listed in `AGENTS.md` → "Things To Avoid".

## Discovery modes

| Mode | Trigger | When to use |
|------|---------|-------------|
| **On-demand** (`description`) | Agent detects task relevance from keywords | Task-based guidance (migrations, refactors, API work) — no `applyTo` needed |
| **Explicit** (`applyTo`) | File matching the glob is in context | File-based guidance (language standards, framework rules) |
| **Manual** | User picks via `Add Context` → `Instructions` | Ad-hoc attachment |

A file can use **both** modes — `description` for on-demand + `applyTo` for auto-attach. All 7 existing files in this repo use both.

## Description — the "Use when..." pattern

Start the description with **"Use when..."** and pack it with keywords the agent would associate with the task:

```yaml
# ❌ Vague — won't be discovered
description: "Helpful coding tips for the project"

# ✅ Keyword-rich — discoverable
description: "Use when writing or editing RP2040 firmware under firmware/. Covers two-mode CMakeLists (host CLI unit tests with Catch2 vs Pico SDK build), main.cpp outer while(true) re-establishes transport, microros.hpp per-peripheral struct pattern, publisher/subscriber creation idioms, callback C-style cast, device classes, MAGNETIC_DECLINATION/HEADING_OFFSET constants."
```

Include: trigger verbs (writing, editing, reviewing, creating), specific file paths, class/function names, and distinctive terms.

## `applyTo` glob patterns

| Pattern | Matches |
|---------|---------|
| `**/*.py` | All Python files, anywhere |
| `ros_packages/**` | Everything under `ros_packages/` |
| `**/*.cpp, **/*.hpp, **/*.h, **/*.cc, **/CMakeLists.txt` | All C++ + CMake files |
| `.devcontainer/**, scripts/**, install.sh, .github/workflows/**` | Multiple dirs + a specific file |
| `ground_station/src/widgets/map_widget/frontend/**` | A specific subdirectory |

> ⚠️ **Avoid `applyTo: "**"`** — it loads the instruction into context on EVERY file request, even when irrelevant. Burns context window. Use specific globs unless the instruction truly applies to all files.

## Body structure — one concern per file

```markdown
---
description: "Use when..."
applyTo: "..."
---

# <Domain> Conventions

<1-2 sentence overview.>

## <Section: config / workflow>

<Verbatim code blocks showing the canonical pattern.>

## <Section: key API / patterns>

<Naming, signatures, gotchas with ⚠️ markers.>

## Things to avoid

- <Bullet list of anti-patterns specific to this domain.>
```

### Principles

1. **One concern per file** — don't mix testing + styling + API design in one file. Split into separate `.instructions.md` files.
2. **Show, don't tell** — verbatim code blocks beat prose. Copy the canonical pattern from the actual codebase.
3. **Concise and actionable** — the instruction shares the context window. Keep focused on what the agent needs to know to not make mistakes.
4. **Flag gotchas with ⚠️** — drifts, bugs, non-obvious invariants the agent would otherwise get wrong.
5. **Link, don't duplicate** — reference `AGENTS.md` or other docs rather than copying content.

## Full template

```markdown
---
description: "Use when writing or editing <what>. Covers <key topics, class names, gotchas>."
applyTo: "<glob patterns>"
---

# <Domain> Conventions

<Overview sentence.>

## <Config / setup>

<Verbatim config file or command.>

## <Key pattern>

```<lang>
<verbatim code from the codebase showing the canonical pattern>
```

> ⚠️ <Gotcha description.>

## Things to avoid

- <Anti-pattern 1>
- <Anti-pattern 2>
```

## Creation checklist

1. **Pick the domain** — one concern, one stack area.
2. **Choose `applyTo`** — specific glob(s) matching the files the instruction governs. Use comma-separated string form.
3. **Write `description`** — "Use when..." + keywords + key class/function/gotcha names.
4. **Write the body** — overview → config → verbatim code blocks → gotchas (⚠️) → things to avoid.
5. **Verify frontmatter** — `---` on line 1, `---` on line 4, `description:` present, `applyTo:` is a string (not array).
6. **Update `AGENTS.md`** — add the new file to the "Instruction Files" table in AGENTS.md.
7. **Run `get_errors`** on the new file to confirm no validation issues.
8. **Commit in the same PR as the code change** — never defer instruction updates to a later PR. Stale instructions mislead every subsequent agent session and teammate.

## Maintenance checklist (run after ANY codebase change)

Before marking a task complete, ask: **"Did this change invalidate anything in `.github/instructions/` or `AGENTS.md`?"** If yes:

1. Identify which instruction file(s) cover the changed code (match by `applyTo` glob + domain).
2. `read_file` the affected instruction file and the current source side-by-side.
3. Update every code block, class name, function signature, file path, and topic name to match the new source.
4. Remove any ⚠️ gotcha notes that have been fixed in code (don't leave stale warnings).
5. Add ⚠️ notes for any newly-discovered gotchas.
6. Update the `description` frontmatter if the scope or keywords shifted.
7. Update the "Instruction Files" table in `AGENTS.md` if `applyTo` or the domain summary changed.
8. Run `get_errors` on every edited file.
9. Commit the instruction updates **in the same PR** as the code change.

## Anti-patterns

- **⚠️ Stale instructions** — the #1 anti-pattern. Leaving an instruction file or AGENTS.md unchanged after modifying the code it describes. Stale instructions cause the agent to apply outdated patterns and reference removed APIs. Always update instructions in the same PR as the code change.
- **⚠️ Deferring instruction updates to a separate PR** — separate PRs get forgotten, sit unreviewed, or conflict. Update instructions in the same PR as the code change, always.
- **⚠️ Trusting memory over source** — when editing an instruction file, always `read_file` the current source to verify class names, signatures, and paths. Memory drifts; source doesn't.
- **Vague descriptions** — "Tips for Python" won't be discovered. Use "Use when writing ROS 2 Python nodes, editing StateManager, or working with telemetry topics."
- **YAML array `applyTo`** — silently fails. Always comma-separated string.
- **`applyTo: "**"` with narrow content** — burns context on every request.
- **Duplicating docs** — link to README/AGENTS.md instead of copying.
- **Mixing concerns** — testing + styling + API design in one file. Split them.
- **No code examples** — prose-only instructions are less effective than verbatim canonical patterns.
- **No "Things to avoid" section** — the agent needs to know what NOT to do, not just what to do.
- **Leaving fixed ⚠️ gotchas in place** — if the codebase bug was fixed, remove the ⚠️ note. Stale warnings are noise.
- **Forgetting to update AGENTS.md** — the new instruction file should be listed in the AGENTS.md "Instruction Files" table.
