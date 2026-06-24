# GolfBot 9000 — Handoff Document
**CDIO Group 24 | Branch: `refactor` | Date: 2026-06-24**

---

## What the robot does

Autonomous EV3 Mindstorms robot that picks up ping-pong balls (white + orange) from a field and deposits them in scored holes in the wall. A host PC camera watches overhead, detects the robot's position via ArUco marker, plans an A\* route to all balls, and sends one move command at a time via TCP. After every single move the robot's real position is re-measured and the next command is recalculated — this is **closed-loop navigation**.

---

## How to start a run (school network)

### 1. Verify EV3 is reachable
```
ping 10.56.138.36
```
Home IP is `192.168.2.13` — ping both and use whichever responds.

### 2. Start both EV3 processes via restart.sh
```
ssh robot@10.56.138.36 "bash ~/CDIO-Mindstorms/robot/restart.sh"
```
This starts **two** processes on the EV3:
- `ev3_server.py` — TCP bridge, listens on port 9999, translates text commands → PyBricks motor calls
- `main.py` — motor controller, reads from TCP, drives DriveBase + gate + lift

**Never start main.py alone via brickrun.** Without ev3_server.py, the host loop has nothing to connect to and the robot won't move.

### 3. Start the host loop
```
python host/loop.py --profile school --camera 1
```
Camera index 1 is the overhead USB camera. If detection is broken try `--camera 0`.

### 4. Verify everything is running
```
ssh robot@10.56.138.36 "fuser 9999/tcp && ps aux | grep -E 'ev3_server|main.py' | grep -v grep"
```
Should show a PID for port 9999 and both process names.

### 5. Check logs after a run
```
ssh robot@10.56.138.36 "cat /tmp/main.log"
ssh robot@10.56.138.36 "cat /tmp/ev3_server.log"
```

---

## Codebase map

```
host/
  loop.py          ← Main closed-loop controller (run this on the PC)
  pathfinding.py   ← A* planner + FieldPlanner + route optimizer
  detection.py     ← Ball detection (Roboflow YOLO + HoughCircles fallback)
  config.py        ← ALL tuneable constants (distances, speeds, thresholds)
  nav.py           ← follow_path() — pose → command generator
  link.py          ← TCP/Bluetooth/Sim communication layer
  ball_preview.py  ← Diagnostic preview: shows planned route without running robot

robot/
  main.py          ← PyBricks micropython on EV3: executes TURN/FWD/GATE_OPEN etc.
  ev3_server.py    ← TCP bridge: receives text commands, calls main.py handlers
  restart.sh       ← Single-command startup script (start BOTH processes in order)
```

---

## Architecture

```
Host PC (loop.py)
  ├─ Camera → OpenCV frame
  ├─ detect_robot() → ArUco pose (x, y, heading_deg)
  ├─ BallDetector.analyze_course() → ball positions, field bounds, hole markers
  ├─ FieldPlanner.plan_trips() → A* waypoint list
  ├─ follow_path() → TURN:N / FWD:N text commands
  └─ TCPLink.send_and_wait("TURN:30") → blocks until EV3 replies "OK"

EV3 (ev3_server.py + main.py)
  ├─ ev3_server.py: TCP socket on port 9999, reads text, calls EV3NavController methods
  └─ main.py: DriveBase, gate motor (Port D), lift motor (Port C), gyro (Port S1)
```

**Command protocol:** host sends `TURN:30`, `FWD:65`, `GATE_OPEN:90`, `GATE_CLOSE:90`, `LIFT_UP:45`, `SPEED:300`, `STOP`. EV3 replies `OK` when done.

---

## Ball detection

Primary: **Roboflow YOLO** (`ping-pong-finder-w6mxk/9`). API key is in `.env` at repo root (`ROBOFLOW_API_KEY=...`). If `.env` is missing, detection falls back to HoughCircles (much less reliable).

ArUco markers:
- `id=0` — robot marker (4×4 dictionary)
- `id=1` — left wall hole
- `id=2` — right wall hole

Hole markers are used to set the dropoff position and the approach angle. If neither is visible, the fallback dropoff is 5% from the left edge, 50% down.

---

## Pathfinding (host/pathfinding.py)

`FieldPlanner.plan_trips()` is the main entry point. It:
1. Builds an obstacle grid (walls + centre cross obstacle + robot footprint inflation)
2. Chooses ball visit order via brute-force TSP (≤9 balls = optimal; more = nearest-neighbour)
3. For each ball, calls `_pickup_seg()`:
   - **Wall/corner/centre-adjacent balls**: computes a forced approach direction (perpendicular to wall, diagonal for corners, inward-radial for centre) → places approach point → A\* from current pos to approach point → straight final segment
   - **Open-field balls**: runs A\* to ball, walks back `BALL_APPROACH_MM=400mm` along that path to find approach point (guarantees no obstacle on final straight)
4. For dropoff: approach_wp is computed geometrically as `hole_pos - 100px * face_vector`. A\* targets approach_wp; raw hole position is appended as the final waypoint.

Key constants in `config.py`:
- `BALL_APPROACH_MM = 400` — how far back from ball to start the final straight approach
- `TRIM_MM = 135` — distance trimmed from the END of each driven segment (the robot's ArUco is not at the front, so it stops 135mm before the waypoint to put the gate tray on the ball)
- `ARRIVE_PX = 20` — pixel radius for "arrived at waypoint"
- `MAX_STEP_MM = 20` — max single forward command; must satisfy `MAX_STEP_MM * px_per_mm < ARRIVE_PX`

---

## Gate & lift

**Gate (Port D motor):** Opens sideways to let balls into the tray. On every ball pickup waypoint, host sends `GATE_OPEN:90` then `GATE_CLOSE:90`. At dropoff, sends `GATE_OPEN:45` (partial, to avoid slamming the wall).

**Lift (Port C motor):** Tilts the tray to pour balls into the hole. At the final dropoff waypoint, host sends `LIFT_UP:45`.

**Gate homing on startup:** As of the latest commit, `main.py` runs the gate motor to the open stop via `run_until_stalled` before resetting the encoder to 0. This prevents the "slam on restart" bug where the gate was already physically closed and `GATE_CLOSE:90` drove it 90° further into the stop.

---

## Known issues / what still needs work

### 1. Path overlay looks like zigzag crossing lines
The display in `loop.py` draws all A\* path waypoints as connected lines. Because the A\* routes around the centre obstacle, segments going on different sides of the obstacle cross each other visually. The **robot navigation is correct** underneath — the visual crossings are a rendering artefact, not actual crossed paths.

### 2. Ball circles not drawn in loop.py overlay
`source.ball_pxs` is drawn as coloured circles. This list is populated at planning time and depleted as balls are collected. If the robot hasn't planned yet (just started) or the plan was made while no balls were visible, no circles appear. Not a bug — wait for the first plan cycle.

### 3. Approach alignment for balls near the centre obstacle
`_forced_approach_dir` uses `ctr_threshold = center_clearance_px + 80px`. If this is too aggressive it forces approach directions for balls that are actually reachable from any angle. If too small, balls near the cross get approached from the wrong side. Tune `CENTER_OBSTACLE_EXTRA_PX` in `config.py` if balls near the cross are being missed.

### 4. Turn overshoot
`TURN_COAST_DEG = 3.0`, `TURN_SLOPE = 1.0` — calibrated from a few observed turns. If the robot consistently over/under-rotates, adjust these in `config.py`. The compensation formula is in the comment above those constants.

### 5. Robot capacity
Hardcoded to `capacity=8` in `loop.py`. The tray physically holds ~8 balls. Change in `loop.py` at the `plan_trips(... capacity=8 ...)` call.

---

## Calibration constants (measured, do not guess)

| Constant | Value | Meaning |
|---|---|---|
| `ACTUAL_PX_PER_MM` | 1.39 | Camera pixels per mm at field height |
| `FORWARD_CMD_SCALE` | 3.2288 | EV3 FORWARD command unit → mm conversion |
| `ROBOT_WIDTH_MM` | 240 | Track-to-track width |
| `ROBOT_LENGTH_MM` | 320 | Front bumper to back |
| `ARUCO_FROM_BACK_FRAC` | 0.344 | ArUco marker is 110/320 = 34.4% from back |
| `TRIM_MM` | 135 | Stop this far before ball so gate tray reaches it |
| `TURN_SLOPE` | 1.0 | Actual/commanded turn ratio |
| `TURN_COAST_DEG` | 3.0 | Motor coasts this far after stop signal |

---

## Useful SSH commands

```bash
# Start both processes (always use this, not brickrun directly)
ssh robot@10.56.138.36 "bash ~/CDIO-Mindstorms/robot/restart.sh"

# Kill everything and check status
ssh robot@10.56.138.36 "bash ~/CDIO-Mindstorms/robot/restart.sh"  # restart handles kill+start

# View live main log
ssh robot@10.56.138.36 "tail -f /tmp/main.log"

# Pull latest code on EV3
ssh robot@10.56.138.36 "cd ~/CDIO-Mindstorms && git stash && git pull"

# Check battery
ssh robot@10.56.138.36 "grep BATT /tmp/main.log | tail -1"

# Verify both processes are up
ssh robot@10.56.138.36 "fuser 9999/tcp && ps aux | grep -E 'ev3_server|main.py' | grep -v grep"
```

---

## Diagnostic tools

```bash
# Preview planned route without connecting to EV3 (shows path, ball order, A* grid)
python host/ball_preview.py

# Just print robot ArUco pose (no commands sent)
python host/loop.py --probe

# Plan from one camera frame, show path, then exit
python host/loop.py --plan-only
```

---

## Branch state

- Branch `refactor` is the active development branch. It has all the closed-loop navigation code.
- Branch `main` is older/stable but missing the latest pathfinding improvements.
- When ready to ship, merge `refactor` → `main`.

---

## If things go wrong

| Symptom | Fix |
|---|---|
| Host loop can't connect to EV3 | Ping `10.56.138.36`. Run `restart.sh`. Check `fuser 9999/tcp` on EV3 |
| Robot moves but ignores commands / no ACK | ev3_server.py not running. Run `restart.sh` |
| Gate slams on startup | Gate was closed before restart. Fixed by homing in latest main.py. If still happens: manually open gate before running `restart.sh` |
| `brickrun: console-runner-service is busy` | Press physical back button on EV3 to stop the running program |
| Robot spinning/oscillating at a waypoint | Tune `ARRIVE_PX` up slightly or `MAX_STEP_MM` down in config.py |
| Balls not detected | Check `.env` has `ROBOFLOW_API_KEY`. Try `ball_preview.py` to isolate |
| `git pull` blocked on EV3 | `git stash` first, then `git pull` |
| Push rejected (non-fast-forward) | `git pull --rebase`, resolve conflicts, then push |
