# Closed-Loop Navigation — Handoff

**Branch:** `closed-loop` (pushed to `origin/closed-loop`, latest commit `b9ce69e`).
All changes are committed and pushed.

---

## Where to pick up next session

1. **Charge battery before anything else.** Previous session ended from battery drain mid-run.

2. **Verify CENTER_OBSTACLE_EXTRA_PX is not too large.** With the current value of 40px,
   `--plan-only` reported "Skipping 4 ball(s) inside obstacles/walls" out of 5 detected.
   That may mean balls placed near the X are legitimately unreachable — or it may mean
   40px is over-inflating the zone. Check visually in the plan-only window:
   - If skipped balls are clearly in open space → lower `CENTER_OBSTACLE_EXTRA_PX` to 20.
   - If skipped balls are genuinely close to the X arms → keep 40.

3. **Verify ball pickup with new ARRIVE_PX=12 and BALL_GATE_THRESHOLD_PX=80.**
   These were the main bug fixes this session but have not been tested with a charged battery.

4. **Watch for gate motor timeout at dropoff.** Previous run timed out on `GATE_OPEN:90`
   at the dropoff step. Likely caused by battery being nearly dead. If it recurs on a
   fresh battery, investigate `run_angle` blocking in `robot/main.py` (see issue 4 below).

---

## What is this

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
| `tools/vision_detector.py` | Ball detection (Roboflow YOLO + HoughCircles fallback), `detect_robot()` via ArUco. |
| `tools/tools_path_planner.py` | `FieldPlanner` — A* path planning + ball routing. |
| `tools/vision_config.py` | Shared constants: Roboflow API key/URL/model, camera index, fallback obstacle radius. |
| `robot/ev3_server.py --tcp` | Python3 TCP bridge on EV3, listens port 9999, writes to file bridge. |
| `robot/main.py --follow` | PyBricks executor: reads file bridge one command at a time, acks each with DONE. |

**Two processes on the EV3** because motor control (PyBricks) and networking (Python3)
can't run in the same process. They communicate through a sequence-numbered file bridge:
`/home/robot/cl_cmd.txt` and `/home/robot/cl_ack.txt`.

---

## How to run

### 1. Start EV3 processes

```bash
ssh robot@10.56.138.36
cd /home/robot/CDIO-Mindstorms
git pull

# Both in background:
nohup python3 -u robot/ev3_server.py --tcp > /tmp/ev3.log 2>&1 & disown
sleep 1
nohup brickrun -r -- pybricks-micropython /home/robot/CDIO-Mindstorms/robot/main.py --follow > /tmp/main.log 2>&1 & disown
```

Wait ~8 seconds, then verify:
```bash
ssh robot@10.56.138.36 "tail -4 /tmp/main.log && fuser 9999/tcp && echo PORT_OPEN"
```
Expect `[EV3] Follow loop ready` and `PORT_OPEN`.

### 2. PC controller

```bash
cd tools/
python -u closed_loop_controller.py --tcp 10.56.138.36
```

Flags:
- `--probe` — show live ArUco pose only (verify tracking before a run)
- `--plan-only` — warm up YOLO (~2s), plan from one frame, show path overlay
- `--sim` — full control-loop test with no hardware (should always PASS)
- `--tcp <ip>` — TCP over WiFi (use for all real runs)
- `--camera N` — override camera index

Abort mid-run: press **`q`** in the camera window.

### 3. Stop the robot safely

```bash
ssh robot@10.56.138.36 "fuser -k 9999/tcp"
```

**NEVER** `pkill -f ev3_server` — matches the SSH shell and kills your session.
**NEVER** `pkill -9 brickrun` or `pkill -9 pybricks-micropython` — crashes the EV3 OS,
triggers a full reboot (60+ seconds to recover).

### 4. Restart EV3 processes between runs

Run as **two separate SSH commands** — chaining kill + start in one session causes the
kill to terminate the shell before the start commands execute:

```bash
ssh robot@10.56.138.36 "fuser -k 9999/tcp 2>/dev/null; sleep 1"
ssh robot@10.56.138.36 "nohup python3 -u /home/robot/CDIO-Mindstorms/robot/ev3_server.py --tcp > /tmp/ev3.log 2>&1 & disown; sleep 1; nohup brickrun -r -- pybricks-micropython /home/robot/CDIO-Mindstorms/robot/main.py --follow >> /tmp/main.log 2>&1 & disown"
# Then: ssh robot@10.56.138.36 "sleep 8 && tail -4 /tmp/main.log"
```

**Restart the EV3 only when:**
- A TCP command timed out (connection may be in a bad state)
- The gate motor stalled (restart resets motor position to 0)
- `main.py --follow` printed an error and exited

### 5. Common mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Forgot `--tcp 10.56.138.36` | "No Bluetooth devices found!" and exit | Always pass `--tcp <ip>` |
| `python3 -u` missing | Log stays empty until run ends | Always use `-u` (unbuffered) |
| `pkill -f ev3_server` over SSH | SSH session dies immediately | Use `fuser -k 9999/tcp` |
| `pkill -9 brickrun` | EV3 crashes, 60s reboot | Never pkill -9 EV3 processes |
| Gate motor stalled mid-run | TCP timeout on GATE_OPEN/CLOSE | Restart EV3 to reset motor position |
| Ran from repo root | `ModuleNotFoundError: tools_path_planner` | Run from **inside** `tools/` directory |
| Camera window grey/frozen | Another process holds the camera | `fuser /dev/video1` and kill it |

---

## Connection details

- **Robot IP**: `10.56.138.36` (DHCP — check EV3 screen if SSH fails; `arp -a | grep -i lego`)
- **SSH key**: `id_ed25519` from this PC is in the robot's `~/.ssh/authorized_keys`
- **Camera**: index set in `vision_config.py`. Override with `--camera N` if needed.

---

## Key tunables (top of `closed_loop_controller.py`)

| Constant | Value | Why |
|---|---|---|
| `ARRIVE_PX` | 12.0 | Must exceed planner step in px (≈7px). **Reduced from 35 this session** — old value put nose up to 98mm from ball. |
| `MAX_STEP_MM` | 20 | One step ≈ 28px actual. Constraint: `MAX_STEP_MM × actual_px_per_mm < ARRIVE_PX`. |
| `MIN_STEP_MM` | 10 | Smallest nudge sent. |
| `TURN_TOL_DEG` | 8.0 | **Reduced from 12 this session.** TURN_COAST=3°, so 8° leaves 5° command headroom. |
| `TURN_COMMIT_DEG` | 45 | Anti-oscillation — won't turn again immediately after one turn. |
| `TURN_SLOPE` | 1.0 | Turn compensation multiplier. |
| `TURN_COAST_DEG` | 3.0 | Turn coast overshoot correction. |
| `FORWARD_CMD_SCALE` | 3.2288 | Physical mm → EV3 command unit. |
| `MAX_POSE_MISS` | 60 | Abort after this many consecutive frames with no marker. |
| `REPLAN_PX` | 150.0 | Force replan when robot is >150px off target. |
| `REPLAN_EVERY_N` | 8 | Also replan every 8 forward steps regardless. |
| `DENSIFY_GAP_PX` | 20.0 | **Reduced from 30 this session.** More heading corrections on ball approach. |
| `GATE_OPEN_DEG` | 90 | Motor angle for gate open. |
| `GATE_CLOSE_DEG` | 90 | Motor angle for gate close. |
| `BALL_GATE_THRESHOLD_PX` | 80 | **Raised from 40 this session.** Must exceed half-length trim offset (~66px). |
| `CENTER_OBSTACLE_EXTRA_PX` | 40 | **New this session.** Extra clearance added to X obstacle radius in A*. May need reducing. |

---

## Issues encountered this session and how they were fixed

### 1. Gate never fired at ball waypoints

**Root cause**: `BALL_GATE_THRESHOLD_PX = 40` was less than the half-length trim offset (~66px).
Ball waypoints are trimmed back by `ROBOT_LENGTH_MM/2 × px_per_mm ≈ 66px` so the nose
stops on the ball, not the robot centre. `_near_ball()` checked whether the **trimmed**
waypoint was within 40px of a detected ball — it never was, so gate never opened or closed.

**Fix**: raise `BALL_GATE_THRESHOLD_PX` to 80.

---

### 2. YOLO cache empty at planning time — 0 balls every run

**Root cause**: YOLO background thread enforces `YOLO_CALL_INTERVAL = 1.0s` before first
API call. Old warmup ran 5 frames × 50ms = 250ms — cache was always empty at plan time.
Same bug in `run_plan_only` with no warmup at all. Also: `ball_confirm_frames = 4` requires
4 YOLO hits to confirm a ball, but YOLO fires at most once per second, so the tracker
never accumulated enough hits during warmup.

**Fix in `run_live` and `run_plan_only`**:
```python
detector.ball_confirm_frames = 1  # single YOLO result is enough for planning
yolo_mode = getattr(detector, '_roboflow_client', None) is not None
deadline = time.monotonic() + (detector.YOLO_CALL_INTERVAL + 0.5)
while time.monotonic() < deadline or (yolo_mode and not detector._yolo_cached_result):
    f = source.grab()
    if f: detector.analyze_course(f)
    time.sleep(0.05)
```

---

### 3. Robot colliding with red X obstacle

**Root cause**: `minEnclosingCircle` on the thin X arms underestimates the true obstacle
radius. A* routed the robot too close to the arms.

**Fix**: `CENTER_OBSTACLE_EXTRA_PX = 40` added to X radius before A* planning.
Adds ~112mm extra clearance. May be worth reducing to 20 if too many reachable balls
end up classified as inside the obstacle.

---

### 4. Gate motor timeout at dropoff

**Symptom**: `TCP timeout waiting for DONE (cmd: GATE_OPEN:90)` at the final dropoff step.
**Likely cause**: battery critically low — motor stalled or ran too slowly to finish within
`ACK_TIMEOUT = 25s`.
**What to try next**: test with a freshly charged battery. If timeout recurs:
- Check `robot/main.py` gate command — `run_angle()` blocks indefinitely on stall.
- Add `stop_then_coast` or reduce gate angle if the motor hits a physical stop.

---

### 5. pkill -9 brickrun crashed the EV3 OS

Forcefully killing brickrun or pybricks-micropython crashes the EV3 Linux OS and triggers
a full reboot. Only use `fuser -k 9999/tcp` to stop processes gracefully.

---

### 6. ev3_server.py log empty under nohup

Python output buffering held stdout until process exit.
**Fix**: always launch with `python3 -u` (unbuffered).

---

### 7. Chaining kill + start in one SSH command

`fuser -k 9999/tcp && nohup ...` killed the shell before the nohup command ran.
**Fix**: two separate SSH calls — one to kill, one to start.

---

## Gate control details

Gate motor is **Port D**. Sequence per ball:

1. **4 waypoints before ball** (~90mm away): `GATE_OPEN:90` — gate arm deploys.
2. **Robot drives** ~90mm to the ball with gate open.
3. **On arrival at ball waypoint**: `GATE_CLOSE:90` — retains ball.
4. **Repeat** for each subsequent ball.
5. **At last waypoint** (dropoff): `GATE_OPEN:90` — releases all balls.

`gate_state` is updated in the replan callback so gate logic stays correct after replanning.

---

## Calibration (measured — do not guess)

| Parameter | Value | How measured |
|---|---|---|
| `FORWARD_CMD_SCALE` | 3.2288 | FORWARD:161 → 50mm measured. |
| `TURN_SLOPE` | 1.0 | Recalibrated from live turn log (cmd→actual: 104→108, 61→63, 156→158). |
| `TURN_COAST_DEG` | 3.0 | Recalibrated from same run (was 8.0). |
| Turn speed | 45°/s | Slow enough to avoid gyro coast overshoot. |
| Actual camera px/mm | ~1.39 | FORWARD:161 (50mm) moved 69px in image. |
| Planner `px_per_mm` | ~0.36 | Printed at startup. Lower than actual — irrelevant, step dominated by MAX_STEP_MM. |

---

## Color calibration (if balls are missed)

```bash
cd tools/
python vision_app.py --calibrate
```
- **N** — cycle WHITE / ORANGE / RED (auto-saves current color)
- **S** — save to `color_ranges.json`
- **Q** — quit

The calibration UI applies CLAHE normalisation to the V channel before thresholding,
matching exactly what the detector sees. Commit any updated `color_ranges.json`.

---

## Validated on real hardware

- ArUco pose detection: ≤1px / ≤0.4° jitter stationary, 19/20 detection rate
- TCP transport + file bridge: PING/PONG handshake and DONE acks confirmed working
- SPEED / FORWARD / TURN / GATE_OPEN / GATE_CLOSE: all execute and ack
- Turn compensation: ±1–2° accuracy after recalibration
- Full course runs: multiple completed, no navigation oscillation
- `--sim` passes every run

**Not yet validated with a charged battery (changed this session):**
- `ARRIVE_PX=12` — ball pickup accuracy
- `BALL_GATE_THRESHOLD_PX=80` — gate fires at ball waypoints
- `CENTER_OBSTACLE_EXTRA_PX=40` — X clearance without over-excluding balls

---

## Architecture notes

- **Half-length trim**: ball waypoints trimmed back by `ROBOT_LENGTH_MM/2 × px_per_mm ≈ 66px`
  so the nose stops on the ball. `BALL_GATE_THRESHOLD_PX` must exceed this offset.

- **YOLO background thread**: Roboflow API called at most once per `YOLO_CALL_INTERVAL=1.0s`.
  Cache is empty until first API response. Always warm up before planning. Use
  `ball_confirm_frames=1` during planning (can't accumulate multiple YOLO results in <1s).

- **px_per_mm mismatch**: planner's `px_per_mm ≈ 0.36` is ~4× lower than actual (~1.39).
  Doesn't matter — `MAX_STEP_MM` caps the step. Key constraint: `MAX_STEP_MM × actual_px_per_mm < ARRIVE_PX`.

- **Dense waypoints vs. replan**: densification corrects small heading drift mid-segment.
  Replan handles large drift or newly-visible balls. They're complementary.
