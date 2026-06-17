# Closed-Loop Navigation — Handoff

**Branch:** `closed-loop` (pushed to `origin/closed-loop`, latest commit `85cd451`).
All changes below are committed and pushed.

---

## What this is

Replaces open-loop dead-reckoning with a **host-driven control loop**: the PC watches
the robot through the overhead camera and sends **one small move at a time**,
re-measuring the robot's real pose after every move.

```
observe robot pose (ArUco)  ->  compare to next waypoint  ->  send ONE move
        ^                                                          |
        +----------------  wait for the robot's DONE  <-----------+
```

---

## Architecture

| File | Role |
|------|------|
| `tools/closed_loop_controller.py` | Main PC controller: camera pose source, control loop, gate logic, link classes. |
| `tools/vision_detector.py` | `detect_robot(frame)` → `(x, y, heading_deg)` via ArUco (DICT_4X4_1000, id 0). |
| `tools/tools_path_planner.py` | `FieldPlanner` — A* path planning + ball routing. |
| `robot/ev3_server.py --tcp` | Python3 TCP bridge on the EV3, listens port 9999, writes to file bridge. |
| `robot/main.py --follow` | PyBricks executor: reads file bridge one command at a time, acks each with DONE. |

**Two processes on the EV3** because motor control (PyBricks) and networking (Python3)
can't run in the same process. They communicate through a sequence-numbered file bridge:
`/home/robot/cl_cmd.txt` and `/home/robot/cl_ack.txt`.

---

## How to run

### 1. Robot processes

```bash
ssh robot@10.56.138.36
cd /home/robot/CDIO-Mindstorms
# Update to latest code:
git pull

# Terminal 1 (TCP bridge):
python3 robot/ev3_server.py --tcp

# Terminal 2 (motor executor) - note brickrun needs -r flag:
brickrun -r -- pybricks-micropython /home/robot/CDIO-Mindstorms/robot/main.py --follow
```

Or run both in background (nohup):
```bash
nohup python3 /home/robot/CDIO-Mindstorms/robot/ev3_server.py --tcp > /tmp/ev3.log 2>&1 &
sleep 1
nohup brickrun -r -- pybricks-micropython /home/robot/CDIO-Mindstorms/robot/main.py --follow > /tmp/main.log 2>&1 &
```

Wait for `[EV3] Follow loop ready` in `/tmp/main.log` before starting the PC controller.

### 2. PC controller

```bash
cd tools/
QT_QPA_PLATFORM=xcb python3 -u closed_loop_controller.py --tcp 10.56.138.36
```

Flags:
- `--probe` — show live ArUco pose only (verify tracking before a run)
- `--plan-only` — plan from one frame and show the path overlay
- `--sim` — full control-loop test with no hardware (regression test, should always PASS)
- `--tcp <ip>` — TCP over WiFi (use this for all real runs)
- `--camera N` — override camera index (default: `CAMERA_INDEX=1` in `vision_config.py`)

Abort mid-run: press **`q`** in the camera window.

### 3. Stopping the robot safely

```bash
fuser -k 9999/tcp   # kills ev3_server, stops command flow
```

**NEVER** use `pkill -f ev3_server` over SSH — the pattern matches your SSH shell's
command line and kills the session. Use `fuser -k 9999/tcp` or explicit PIDs.

---

## Connection details

- **Robot IP**: `10.56.138.36` (DHCP — may change if robot reconnects to eduroam)
- **SSH key**: `id_ed25519` from this PC is in the robot's `~/.ssh/authorized_keys`.
  A previous key labelled `claude-cdio` can be removed if present.
- **Camera**: index `1` (`/dev/video1`). Opens with a V4L2 WARN but works via
  fallback. Set `QT_QPA_PLATFORM=xcb` or the cv2 window won't display.
- **Robot git**: repo at `/home/robot/CDIO-Mindstorms`, branch `closed-loop`.
  Run `git pull` before each session to get latest code.

---

## What is VALIDATED on real hardware

- **ArUco pose detection**: ≤1 px / ≤0.4° jitter stationary, 19/20 detection rate
- **TCP transport + file bridge**: handshake (PING/PONG) and DONE acks all work
- **SPEED / FORWARD / TURN / GATE_OPEN / GATE_CLOSE**: all execute and ack on EV3
- **Turn compensation**: ±1–2° accuracy after recalibration (see below)
- **Full course runs**: completed multiple full runs with 4–8 balls detected, periodic
  replanning, no navigation oscillation
- **Ball collection gate**: OPEN before each ball, CLOSE to retain, OPEN at dropoff —
  sequencing confirmed working in live runs. Gate timing still being tuned.
- `--sim` passes every run

---

## Calibration (measured — do not guess)

| Parameter | Value | How it was measured |
|---|---|---|
| `FORWARD_CMD_SCALE` | 3.2288 | `main.py` runs `straight(-v/3.2288)`. FORWARD:161 → 50 mm measured. |
| `TURN_SLOPE` | 1.0 | Recalibrated from 3 live turn log data points (cmd→actual: 104→108, 61→63, 156→158). |
| `TURN_COAST_DEG` | 3.0 | Recalibrated from same run (was 8.0 in previous session). |
| Turn speed | 45 °/s | Slow enough to avoid gyro coast overshoot. Set in `main.py --follow` mode. |
| Actual camera px/mm | ~1.39 | Measured: FORWARD:161 (50 mm) moved 69 px in image. |
| Planner `px_per_mm` | ~0.36 | Printed at startup. Does NOT match actual — step size controlled by `MAX_STEP_MM`. |

---

## Key tunables (top of `closed_loop_controller.py`)

| Constant | Value | Why |
|---|---|---|
| `ARRIVE_PX` | 35.0 | Must be > actual step size in pixels. One 20mm step ≈ 28 px actual < 35. |
| `MAX_STEP_MM` | 20 | **Critical**: step ≈ 28 px actual. Must satisfy `MAX_STEP_MM × actual_px_per_mm < ARRIVE_PX`. Was 50 → caused overshoot/oscillation. |
| `MIN_STEP_MM` | 10 | Smallest nudge sent. |
| `TURN_TOL_DEG` | 12 | Only turn if heading error > this. |
| `TURN_COMMIT_DEG` | 45 | After a turn, must be still badly off to turn again immediately. Anti-oscillation. |
| `TURN_SLOPE` | 1.0 | Turn compensation. |
| `TURN_COAST_DEG` | 3.0 | Turn coast offset. |
| `FORWARD_CMD_SCALE` | 3.2288 | Physical mm → EV3 command unit. |
| `MAX_POSE_MISS` | 60 | Abort after this many consecutive frames with no marker. |
| `REPLAN_PX` | 150.0 | Force replan when robot is >150 px off target. |
| `REPLAN_EVERY_N` | 8 | Also replan every 8 forward steps regardless. |
| `DENSIFY_GAP_PX` | 30.0 | Max gap between consecutive waypoints after densification (≈22 mm). |
| `GATE_OPEN_DEG` | 90 | Motor angle for GATE_OPEN command. |
| `GATE_CLOSE_DEG` | 90 | Motor angle for GATE_CLOSE command. |
| `BALL_GATE_THRESHOLD_PX` | 40 | Waypoint treated as ball pickup if within this many px of a detected ball. |

---

## Issues encountered this session and how they were fixed

### 1. "Lost the robot marker for too long" — aborted after every 1–2 moves

**Root cause**: camera reads were stale. After `send_and_wait()` blocks for 1–2 s,
the next `cap.read()` returns a frame from during the motor command, not after it settled.
ArUco detection on this stale frame failed.

**Fix**: moved camera capture + ArUco detection to a **background daemon thread**
(`CameraPoseSource._grab_loop`). The main control loop just reads the latest
`_latest_pose` from the thread — always fresh, never blocks the display. The background
thread also renders the overlay continuously (no more frozen camera window between moves).

**Critical**: `source.stop()` must call `self._thread.join(timeout=2.0)` **before**
`cap.release()`. Without join, `cap.release()` is called while the thread is still inside
`cap.read()` → segfault / `VIDIOC_DQBUF: Invalid argument` on the second run.

---

### 2. 0 balls detected every run

**Root cause**: `ball_confirm_frames = 7` requires 7 consecutive frame hits before a
ball is confirmed. The planner only called `analyze_course()` once per planning cycle —
so balls always scored 1/7 and were filtered out.

**Fix**:
```python
detector.ball_confirm_frames = 1  # single frame is fine for planning
for _ in range(5):
    f = source.grab()
    detector.analyze_course(f)
    time.sleep(0.05)
```
Warmup loop feeds 5 frames so the tracker has data before planning begins. Set before
every `plan_waypoints()` call (including initial and replan).

---

### 3. wp6 U-turn oscillation — robot spun back and forth at a waypoint

**Root cause**: `MAX_STEP_MM = 50` meant one forward step covered ~69 px in the image
(measured from run log: FORWARD:161 = 50 mm → robot moved 69 px). `ARRIVE_PX = 32`.
Since 69 > 32, the robot **stepped past** the waypoint without triggering arrival. Then it
saw the waypoint was now behind it, turned ~180°, overshot in the other direction, and
repeated. The planner's `px_per_mm ≈ 0.36` is much lower than the actual camera scale
(~1.39 px/mm), so the step calculator always hit `MAX_STEP_MM` — the cap dominated.

**Fix**: reduce `MAX_STEP_MM` to **20** (step ≈ 28 px actual < ARRIVE_PX=35). Increase
`ARRIVE_PX` to **35** for margin. The robot now always detects arrival before
overshooting. Confirmed: no oscillation in subsequent runs.

**Rule**: `MAX_STEP_MM × actual_px_per_mm < ARRIVE_PX`. If you raise MAX_STEP_MM, also
raise ARRIVE_PX. The diagnostic line at startup shows both:
```
px_per_mm: 0.362  MAX_STEP_MM: 20  step_px: 7.2  ARRIVE_PX: 35.0
```
Note `step_px` shows the planner's estimate (7.2); actual step is ~28 px at 1.39 px/mm.

---

### 4. Robot not self-correcting lateral drift

**Root cause**: `REPLAN_PX = 400` meant replanning only happened on extreme drift
(never in practice). The robot navigated waypoint-to-waypoint without refreshing its
view of where balls are.

**Fix**:
- `REPLAN_PX = 150` — replan on moderate drift
- `REPLAN_EVERY_N = 8` — force replan every 8 forward steps regardless
- `DENSIFY_GAP_PX = 30` — insert intermediate waypoints every 30 px so heading errors
  are corrected every ~22 mm of travel, preventing lateral accumulation

---

### 5. Gate motor stalling from duplicate GATE_OPEN commands

**Root cause**: Multiple consecutive waypoints triggered `_near_dropoff()`, each sending
`GATE_OPEN:90`. The motor accumulated 90° per command (0→90→180→270→stall).

**Fix**: track `gate_state['open']`. Each gate command is guarded by the current state:
- GATE_OPEN only sent when `gate_state['open'] == False`
- GATE_CLOSE only sent when `gate_state['open'] == True`
- Dropoff release uses `idx == len(wps) - 1` (exact last waypoint) not proximity — this
  prevents false dropoff triggers when the robot starts near the dropoff zone.

---

### 6. Gate opening too close to ball — pushes ball away

**Root cause**: gate opened 1 waypoint (≈22 mm) before the ball. The gate arm sweeps
forward during the 90° rotation. At 22 mm distance, the arm tip contacts the ball
during the sweep and pushes it sideways instead of scooping it.

**Fix**: look ahead **4 waypoints** (4 × 30 px ≈ 90 mm) when deciding to open the gate:
```python
for look in range(1, min(5, len(wps) - idx)):
    if _near_ball(wps[idx + look]):
        link.send_and_wait("GATE_OPEN:90")
        gate_state['open'] = True
        break
```
Gate is now fully deployed ~90 mm before the robot reaches the ball. Still needs
hardware verification after battery recharge.

---

### 7. pkill killed SSH session

**Cause**: `pkill -f ev3_server` matched the SSH shell's own command line.

**Fix**: always use `fuser -k 9999/tcp` to kill the bridge, or kill by explicit PID.

---

### 8. Camera wrong index

Camera is on `/dev/video1` (index 1). Opens with a V4L2 WARN but works via fallback
backend. Probe: `python3 -c "import cv2; c=cv2.VideoCapture(1); print(c.isOpened())"`.
Set `QT_QPA_PLATFORM=xcb` to enable the cv2 window.

---

## Gate control details

Gate motor is **Port D**. The sequence per ball:

1. **4 waypoints before ball** (~90 mm away): `GATE_OPEN:90` — gate arm deploys.
   Robot is stationary during the 0.45 s opening (send_and_wait blocks).
2. **Robot drives** the ~90 mm to the ball with gate fully open.
3. **On arrival at ball waypoint** (dist < ARRIVE_PX): `GATE_CLOSE:90` — gate closes,
   ball retained.
4. **Repeat** for each subsequent ball.
5. **At last waypoint** (dropoff): `GATE_OPEN:90` — gate opens to release all balls.

`gate_state['ball_pxs']` and `gate_state['dropoff']` are updated in the replan callback
so gate logic stays correct after mid-run replanning.

### What still needs tuning

- **Gate angle** (currently 90°): if balls aren't being retained or released cleanly,
  adjust `GATE_OPEN_DEG` and `GATE_CLOSE_DEG`.
- **Look-ahead** (currently 4 wps ≈ 90 mm): may need adjustment depending on the
  physical gate arm length. More look-ahead → gate opens earlier → safer. But if
  too early, the gate might catch things during transit.
- **Gate motor stall**: if the gate hits a physical stop and stalls, PyBricks'
  `run_angle()` may block indefinitely → TCP timeout. If this happens, restart the EV3
  processes to reset motor position. Future fix: add `wait=False` to `run_angle` calls
  in `main.py` and let the motor coast to position.

---

## Operational checklist before each run

1. Make sure balls are in camera view
2. `git pull` on robot to get latest code
3. Start `ev3_server.py --tcp` and wait for port 9999 to open
4. Start `main.py --follow` and wait for `[EV3] Follow loop ready`
5. Run `python3 -u closed_loop_controller.py --tcp 10.56.138.36` from `tools/`
6. Camera window opens — if robot marker is visible, planning starts automatically
7. Run completes with `Run complete.` — gate opens at dropoff, balls released

---

## Architecture notes / non-obvious design choices

- **Why `_debug_path_segs` not `plan_trips` output?** `plan_trips()` returns EV3
  command strings for open-loop execution. Closed-loop doesn't use those commands —
  it only needs the geometric path. `_debug_path_segs` gives the driven pixel
  polylines that the closed-loop tracks waypoint-by-waypoint.

- **Why px_per_mm mismatch doesn't matter?** The planner's `px_per_mm ≈ 0.36` is
  ~4× lower than the actual camera scale (~1.39). The step calculator always hits
  `MAX_STEP_MM` for any target > 7 px, so `px_per_mm` only affects very close
  targets. The key constraint is `MAX_STEP_MM × actual_px_per_mm < ARRIVE_PX`.

- **Why not fix px_per_mm?** Could, but it's not needed — MAX_STEP_MM already
  bounds the step, making the planner's px_per_mm irrelevant for step sizing.

- **Dense waypoints vs. replan**: densification handles small heading drift mid-segment.
  Replan handles large drift or newly-visible balls. They're complementary.
