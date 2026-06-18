# Repo Refactor Plan — Align with Domain Split

Goal: make `tools/` reflect the domain split in CONTRIBUTING.md so it's obvious where each concern lives and new code has a clear home.

---

## Current state — what each file actually contains

### `tools/` (PC-side)

| File | Lines | What's actually inside |
|---|---|---|
| `closed_loop_controller.py` | ~820 | **God file.** BluetoothLink + TCPLink + SimLink (transport) · CameraPoseSource (camera) · follow_path + _turn_command + tunables (nav) · plan_waypoints + _densify_waypoints + _flatten_segs (planning glue) · gate state machine / on_arrive (pickup) · dropoff release at last waypoint · run_probe/run_live/run_sim entry points |
| `tools_path_planner.py` | ~740 | Path planning only — clean, well-scoped |
| `vision_detector.py` | ~424 | Detection only — clean |
| `vision_app.py` | ~491 | UI / planning app — clean |
| `vision_calibration.py` | ~? | Manual HSV calibration UI |
| `vision_auto_calibrate.py` | ~? | Click-to-sample auto calibration |
| `vision_config.py` | ~78 | Shared constants + color range persistence |
| `vision_simulator.py` | ~519 | Simulator / path replay UI |
| `tools_mission_sender.py` | ~? | Legacy Bluetooth mission sender (open-loop) |
| `color_ranges.json` | — | Calibration data |
| `tools_vision_boundary_detection.py` | 13 | **Stub redirect only** — real code moved to vision_detector/config long ago |
| `generate_aruco_marker.py` | ~? | One-off utility |
| `deploy_robot_only.ps1` | ~? | Windows-only deploy script |
| `requirements.txt` | — | Deps |

### `robot/` (EV3-side — must stay flat, PyBricks constraint)

| File | Lines | What's actually inside |
|---|---|---|
| `main.py` | ~660 | Command dispatcher · open-loop mission execution · closed-loop follow loop · physical calibration constants scattered through comments |
| `ev3_server.py` | ~150 | TCP/Bluetooth bridge, file handshake (cl_cmd.txt / cl_ack.txt) |
| `ev3_receiver.py` | 63 | **Dead** — old Bluetooth receiver, superseded by ev3_server.py |
| `ev3_receiver_sound.py` | ~? | **Dead** — old variant |
| `commands.txt` | — | Active mission |
| `test_tank.py` | ~? | Hardware test |

### Repo root

| File | What it is |
|---|---|
| `nav_controller.py` | **Misplaced** — old screenshot-mode planning script, predates the tools/ split |

---

## Problems

### 1. `closed_loop_controller.py` is a god file
Four unrelated domains in one 820-line file. Adding anything to nav, gate, or transport requires reading all 820 lines to find the right spot.

### 2. `tools/` naming is inconsistent
`tools_*` prefix on some, `vision_*` on others, nothing on a few. No pattern to predict where something lives.

### 3. Dead files are still in the repo
`tools_vision_boundary_detection.py` (stub), `robot/ev3_receiver.py`, `robot/ev3_receiver_sound.py` — none of these are imported or run anywhere in the current codebase.

### 4. `nav_controller.py` is at the repo root
Should be in `tools/` or deleted if fully superseded by `closed_loop_controller.py`.

### 5. Dropoff logic is buried inside `on_arrive` in the god file
The ball release at the dropoff zone is a distinct behaviour (domain 11) but shares a function with pickup gate logic, making it hard to tune independently.

### 6. No start/stop robot scripts exist (domains 12 & 13)
Starting the robot currently means copy-pasting SSH commands from CLOSED_LOOP_HANDOFF.md. Stopping it requires knowing `fuser -k 9999/tcp`. Neither is cross-platform. Both should be a single runnable script.

---

## Proposed split

**Robot side stays flat** (PyBricks constraint — no subdirectories).

**Tools side: split `closed_loop_controller.py` into focused files:**

```
tools/
│
├── nav_links.py          ← NEW: BluetoothLink, TCPLink, SimLink
├── nav_camera.py         ← NEW: CameraPoseSource (background grab thread)
├── nav_core.py           ← NEW: follow_path(), _turn_command(), all tunables
├── nav_gate.py           ← NEW: gate state machine, on_arrive logic, _near_ball, dropoff release
├── nav_planner.py        ← NEW: plan_waypoints(), _flatten_segs(), _densify_waypoints()
├── closed_loop.py        ← RENAME from closed_loop_controller.py — entry points only
│                              (run_probe, run_plan_only, run_live, run_sim, main)
│
├── path_planner.py       ← RENAME from tools_path_planner.py (drop the tools_ prefix)
│
├── vision_detector.py    (unchanged)
├── vision_calibration.py (unchanged)
├── vision_auto_calibrate.py (unchanged)
├── vision_config.py      (unchanged)
├── vision_app.py         (unchanged)
├── vision_simulator.py   (unchanged)
│
├── color_ranges.json     (unchanged)
├── requirements.txt      (unchanged)
├── generate_aruco_marker.py (unchanged)
├── deploy_robot_only.ps1 (unchanged — or move to scripts/)
│
├── tools_mission_sender.py  (keep for now — Bluetooth open-loop path)
│
├── robot_start.py        ← NEW: SSH into EV3, launch ev3_server + main.py --follow (cross-platform)
└── robot_stop.py         ← NEW: SSH into EV3, kill ev3_server + main.py cleanly (cross-platform)
```

**Delete:**
- `tools/tools_vision_boundary_detection.py` — 13-line stub, nothing imports it
- `robot/ev3_receiver.py` — superseded by ev3_server.py
- `robot/ev3_receiver_sound.py` — superseded
- `nav_controller.py` (root) — superseded by closed_loop_controller / vision_app

---

## Migration steps

1. **Delete dead files** (no import changes needed)
   - `tools/tools_vision_boundary_detection.py`
   - `robot/ev3_receiver.py`
   - `robot/ev3_receiver_sound.py`
   - `nav_controller.py`

2. **Split `closed_loop_controller.py`** — extract in this order to keep each step testable:
   - Extract `BluetoothLink`, `TCPLink`, `SimLink` → `nav_links.py`; update import in `closed_loop_controller.py`
   - Extract `CameraPoseSource` → `nav_camera.py`; update import
   - Extract tunables + `follow_path` + `_turn_command` → `nav_core.py`; update import
   - Extract gate logic → `nav_gate.py`; update import
   - Extract `plan_waypoints` + helpers → `nav_planner.py`; update import
   - What remains in `closed_loop_controller.py` is just entry points → rename to `closed_loop.py`
   - Run `python3 -u closed_loop.py --sim` after each step to confirm nothing broke

3. **Create `robot_start.py` and `robot_stop.py`**
   - Use Python's `subprocess` + `ssh` (or `paramiko`) so they work on both Windows and Linux
   - `robot_start.py`: SSH → kill any stale processes → start `ev3_server.py --tcp` → start `main.py --follow` → tail log until `Follow loop ready`
   - `robot_stop.py`: SSH → `fuser -k 9999/tcp` → wait for processes to exit
   - Replace the manual SSH steps in CLOSED_LOOP_HANDOFF.md with a single `python3 tools/robot_start.py`

4. **Rename `tools_path_planner.py` → `path_planner.py`**
   - Update imports in `vision_app.py`, `closed_loop.py` (or `nav_planner.py` after step 2)
   - Update any references in CONTRIBUTING.md / README

---

## What this buys

| Before | After |
|---|---|
| Gate timing bug → search 820 lines | Gate timing bug → open nav_gate.py (~80 lines) |
| Add new transport (USB serial?) → edit god file | Add new transport → add class to nav_links.py |
| Tune nav constants → scroll through mixed code | Tune nav constants → top of nav_core.py |
| "Where do I add ball detection?" → unclear | "Where do I add ball detection?" → vision_detector.py |
| Start robot → copy SSH commands from handoff doc | Start robot → `python3 tools/robot_start.py` |
| Stop robot → remember `fuser -k 9999/tcp` | Stop robot → `python3 tools/robot_stop.py` |
| Tune dropoff → find it inside gate/pickup function | Tune dropoff → nav_gate.py, clearly labelled |
