# Repo Refactor Plan

Goal: one file per domain, no dead files, consistent naming, obvious home for every new piece of code.

---

## Naming conventions

| Prefix | Meaning |
|---|---|
| `nav_*` | Navigation library modules — imported, not run directly |
| `vision_*` | Vision pipeline modules — imported, not run directly |
| `tools_*` | Standalone debug / utility scripts — run directly, not imported |
| *(none)* | Core entry-point files (`closed_loop.py`, `path_planner.py`) |

---

## Target structure

### `robot/` — stays flat (PyBricks constraint: no subdirectories)

One file per domain:

```
robot/
├── main.py         ← EV3NavController: command dispatch, open-loop mission, follow loop
├── ev3_server.py   ← TCP/file bridge between Python3 networking and PyBricks motor control
└── commands.txt    ← active mission (data, not code)
```

### `tools/` — PC-side only

```
tools/
│
│  ── Navigation (closed-loop controller, split from god file) ──
├── nav_config.py              ← NEW: all ~15 tunables and physical constants
├── nav_links.py               ← NEW: BluetoothLink, TCPLink, SimLink
├── nav_core.py                ← NEW: follow_path(), _turn_command()
├── closed_loop.py             ← RENAME: CameraPoseSource, plan_waypoints helpers,
│                                        all entry points (run_probe/run_live/run_sim/main)
│
│  ── Path planning ──
├── path_planner.py            ← RENAME from tools_path_planner.py
│
│  ── Vision pipeline ──
├── vision_detector.py         (unchanged)
├── vision_app.py              (unchanged)
├── vision_calibration.py      (unchanged)
├── vision_auto_calibrate.py   (unchanged)
├── vision_config.py           (unchanged)
├── vision_simulator.py        (unchanged)
│
│  ── Debug / utility scripts ──
├── tools_mission_sender.py    (unchanged — Bluetooth open-loop legacy)
├── tools_generate_aruco.py    ← RENAME from generate_aruco_marker.py
├── robot_start.py             ← NEW: SSH into EV3, start ev3_server + main.py --follow
├── robot_stop.py              ← NEW: SSH into EV3, kill bridge cleanly
│
│  ── Data / config ──
├── color_ranges.json          (unchanged)
├── requirements.txt           (unchanged)
└── deploy_robot_only.ps1      (unchanged — Windows deploy script)
```

### `tests/` — PC-side test and debug scripts

```
tests/
├── test_camera.py        ← keep (useful: camera index probe + live feed)
└── test_navigation.py    ← keep (unit tests for command parsing)
```

---

## Files to delete

### Dead code — nothing imports or runs these

| File | Why dead |
|---|---|
| `tools/tools_vision_boundary_detection.py` | 13-line stub redirect, code moved to `vision_detector.py` and `vision_config.py` |
| `robot/ev3_receiver.py` | Superseded by `ev3_server.py` |
| `robot/ev3_receiver_sound.py` | Superseded by `ev3_server.py` |
| `robot/test_tank.py` | Imports `ev3dev2` — wrong API (project uses PyBricks). Has never worked. |
| `nav_controller.py` (root) | Old screenshot-mode planning script, predates tools/ split, superseded |

### Stale tests

| File | Why stale |
|---|---|
| `tests/test_bluetooth.py` | Queries Windows registry for Bluetooth devices; project moved to TCP |
| `tests/test_sound.py` | 21-line trivial EV3 speaker test, no diagnostic value |

---

## Migration steps

Run `python3 -u closed_loop.py --sim` (or `closed_loop_controller.py --sim` before rename) after each step as the regression gate.

### Step 1 — Delete dead files
No import changes needed. Just delete:
- `tools/tools_vision_boundary_detection.py`
- `robot/ev3_receiver.py`
- `robot/ev3_receiver_sound.py`
- `robot/test_tank.py`
- `nav_controller.py`
- `tests/test_bluetooth.py`
- `tests/test_sound.py`

### Step 2 — Extract `nav_config.py`
Move the tunables block (lines 47–86 of `closed_loop_controller.py`) to `nav_config.py`.
Add `from nav_config import *` (or named imports) at the top of `closed_loop_controller.py`.
Run `--sim` to confirm nothing broke.

This step unblocks everything else: `SimLink` references `FORWARD_CMD_SCALE` and `CameraPoseSource` references `ARRIVE_PX` — both constants must exist in an importable module before those classes can be extracted.

### Step 3 — Extract `nav_links.py`
Move `BluetoothLink`, `TCPLink`, `SimLink` out of `closed_loop_controller.py`.
`SimLink` imports `FORWARD_CMD_SCALE` from `nav_config`.
Add `from nav_links import BluetoothLink, TCPLink, SimLink` in `closed_loop_controller.py`.
Run `--sim`.

### Step 4 — Extract `nav_core.py`
Move `follow_path()` and `_turn_command()` out of `closed_loop_controller.py`.
`nav_core.py` imports its constants from `nav_config`.
Add `from nav_core import follow_path, _turn_command` in `closed_loop_controller.py`.
Run `--sim`.

### Step 5 — Rename `closed_loop_controller.py` → `closed_loop.py`
What remains is: `CameraPoseSource`, `plan_waypoints` + helpers (flatten/densify), and all entry points (`run_probe`, `run_plan_only`, `run_live`, `run_sim`, `main`). Gate logic stays here as closures inside `run_live` — it's 30 lines and tightly coupled to that function's local state.
Run `--sim`.

**Note:** Do NOT extract a separate `nav_gate.py`, `nav_camera.py`, or `nav_planner.py`. The gate logic is closures that close over `gate_state`, `link`, `source`, and `detector`. Extracting them requires either a class or 6+ extra parameters — more churn than value. `CameraPoseSource` and `plan_waypoints` are each only used from one place; a separate file would be one class with one caller.

### Step 6 — Rename `tools_path_planner.py` → `path_planner.py`
Update the import in `closed_loop.py` (and `vision_app.py` if it imports directly).
Run `--sim`.

### Step 7 — Rename `generate_aruco_marker.py` → `tools_generate_aruco.py`
No import dependencies; it's a standalone script.

### Step 8 — Create `robot_start.py` and `robot_stop.py`
Independent of all above steps — can be done anytime.

`robot_start.py`: SSH → kill stale processes → start `ev3_server.py --tcp` → start `main.py --follow` → tail `/tmp/main.log` until `Follow loop ready`.
`robot_stop.py`: SSH → `fuser -k 9999/tcp` → wait for processes to exit.

Use Python's `subprocess` + `ssh` so they work on both Windows and Linux. Replace the manual SSH steps in `CLOSED_LOOP_HANDOFF.md` with a single `python3 tools/robot_start.py`.

### Step 9 — Update docs
- `CONTRIBUTING.md`: rewrite file tree and setup sections to match target structure above
- `README.md`: update Architecture section (closed-loop is primary, not file-based); fix gyro port (S4 → S1)
- `CLOSED_LOOP_HANDOFF.md`: update branch name from `closed-loop` to `refactor`
- Remove `robot/main.py:92` stale comment (`# not working idk why`)

---

## Dependency graph (after refactor)

```
nav_config.py        ← no deps
nav_links.py         ← nav_config
nav_core.py          ← nav_config
path_planner.py      ← vision_config (field dims)
vision_config.py     ← no deps
vision_detector.py   ← vision_config
closed_loop.py       ← nav_config, nav_links, nav_core, path_planner, vision_config, vision_detector
vision_app.py        ← path_planner, vision_config, vision_detector
```

No circular imports.

---

## What this buys

| Before | After |
|---|---|
| Tune a nav constant → read 822-line god file | Tune a nav constant → open `nav_config.py` (~20 lines) |
| Add a new transport → edit god file | Add a new transport → add class to `nav_links.py` |
| "Where does the turn math live?" → unclear | `nav_core.py` |
| Start robot → copy SSH commands from handoff doc | `python3 tools/robot_start.py` |
| Stop robot → remember `fuser -k 9999/tcp` | `python3 tools/robot_stop.py` |
| Dead files confuse new contributors | Every file in the repo has a live purpose |
