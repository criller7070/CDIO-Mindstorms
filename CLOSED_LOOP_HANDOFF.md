# Closed-Loop Navigation — Handoff

Status as of this session. Branch: **`closed-loop`** (pushed to origin). Everything
below is committed.

## What this is

Replaces open-loop dead-reckoning (compute a whole mission, drive it blind — drifts
as per-command errors accumulate) with a **host-driven control loop**: the PC watches
the robot through the overhead camera and sends **one small move at a time**,
re-measuring the robot's real pose after every move.

```
observe robot pose (ArUco)  ->  compare to next waypoint  ->  send ONE move
        ^                                                          |
        +----------------  wait for the robot's DONE  <-----------+
```

## Components (new/changed files)

| File | Role |
|------|------|
| `tools/vision_detector.py` | `detect_robot(frame)` → `(x, y, heading_deg)` from an ArUco marker (DICT_4X4_1000). |
| `tools/closed_loop_controller.py` | The control loop + `BluetoothLink`, `TCPLink`, `SimLink`. Modes: `--probe`, `--plan-only`, `--sim`, `--tcp <ip>`. |
| `tools/generate_aruco_marker.py` | Prints a marker PNG (DICT_4X4_1000) with a wide white quiet zone. |
| `tools/tools_mission_sender.py` | `send_and_wait()` — blocks until the robot acks `DONE`. |
| `robot/main.py` | `--follow` mode: persistent executor, runs ONE command at a time and acks it. |
| `robot/ev3_server.py` | Persistent bridge (`--tcp` for WiFi, default Bluetooth). |

## Transport

**TCP over WiFi** (chosen over Bluetooth — robot is on the network, lower latency).
Robot IP `10.56.138.36`, port `9999`. The robot needs **two** processes because motor
control is `pybricks-micropython` and networking is `python3` — they can't share a
process, so they cooperate through a sequence-numbered file bridge
(`/home/robot/cl_cmd.txt`, `/home/robot/cl_ack.txt`).

## How to run

1. **Marker** (once): `python tools/generate_aruco_marker.py` → `robot_marker_0.png`.
   Print it, mount **flat on top of the robot, top edge toward the FRONT, white border
   intact** (the quiet zone is essential — black-on-black is undetectable).

2. **Connect to the robot** (it's on WiFi at `10.56.138.36`):
   ```
   ssh robot@10.56.138.36
   ```
   Authenticate with the robot's password when prompted (the standard ev3dev
   `robot` account). Open **two** SSH shells — one for each process below — or run
   the first in the background. The repo lives at `~/CDIO-Mindstorms` (i.e.
   `/home/robot/CDIO-Mindstorms`).

   Then, in `~/CDIO-Mindstorms/robot`:
   ```
   python3 ev3_server.py --tcp
   brickrun -- pybricks-micropython /home/robot/CDIO-Mindstorms/robot/main.py --follow
   ```
   > **IP note:** `10.56.138.36` is the robot's current DHCP address — it can
   > change. Confirm it on the EV3 (its WiFi status / `hostname -I` over a known
   > connection) and pass the right IP to the PC controller below.
   >
   > A temporary SSH public key labelled `claude-cdio` was added to the robot's
   > `~/.ssh/authorized_keys` during development; remove that line if you want.
   (Use the **absolute path** for `main.py` — `brickrun` changes the working dir.)
   The bridge takes ~10 s to start on the EV3 (slow ARM); wait until it prints
   `LISTENING` / port 9999 is open before connecting.

3. **On the PC**:
   ```
   python tools/closed_loop_controller.py --tcp 10.56.138.36
   ```
   - `--probe` — just show the live ArUco pose (verify tracking first).
   - `--plan-only` — plan from one frame and show the path.
   - `--sim` — full loop with no hardware (regression test).
   - `--camera N` — override camera index (default `CAMERA_INDEX=1` in `vision_config.py`).
   - Abort: press **`q`** in the window.

## What is VALIDATED on real hardware

- **Perception**: ArUco pose is rock-solid — ≤1 px / ≤0.4° jitter when stationary,
  19/20 detection rate.
- **TCP transport + file bridge**: handshake (`PING`/`PONG`) and `DONE` acks all work.
- **Executor**: `SPEED`, `FORWARD`, `TURN` all run on the EV3 and ack correctly.
- **Turn compensation**: turns land within **±3°** of target (see calibration below).
- **No turn oscillation**: the limit-cycle is fixed.
- `--sim` passes (converges, turns a minority of steps).

## Calibration we measured (important)

- **FORWARD**: `main.py` runs `straight(-value / 3.2288)`, so `FORWARD:v` moves
  ~`v/3.2288` mm. The controller multiplies by `FORWARD_CMD_SCALE = 3.2288` so a
  requested physical step actually travels that far.
- **TURN (follow mode)**: at the default 200 °/s the gyro turn **coasts ~18°** past
  target → oscillation. Follow mode now turns at **45 °/s** (`run_follow_loop` sets
  `self.turn_speed = 45`; only affects follow mode, not missions). Measured response
  then: **actual ≈ 1.05 × commanded + 8°**. The controller inverts this in
  `_turn_command()` (`TURN_SLOPE = 1.05`, `TURN_COAST_DEG = 8`). Verified ±3° on hardware.

## KNOWN ISSUE — where to pick up

The full autonomous run **aborts mid-run with "Lost the robot marker for too long"**
after the first move or two. Stationary detection is fine (19/20), so this is almost
certainly **camera lag / stale frames** (you observed the preview "lagging, almost
crashing"). Last change added `CAP_PROP_BUFFERSIZE = 1` in `_open_camera()` to drop
stale frames — **this has NOT yet been tested in a full run.**

Next steps to try:
1. Re-run and watch whether the marker-loss aborts are gone with the buffer fix.
2. If the preview still lags: the per-step camera read + `imshow` on a shared camera
   may be the bottleneck. Try `show=False` (disable the debug window) in
   `CameraPoseSource`, or run the camera grab on its own thread.
3. If the marker is genuinely lost while moving: confirm the robot isn't driving out
   of the camera's field of view, and that the marker doesn't tilt out of plane.
4. As a cushion, raise `MAX_POSE_MISS` and/or have a pose-miss keep the robot stopped
   and retry rather than aborting so quickly.

## Key tunables (top of `closed_loop_controller.py`)

| Constant | Value | Meaning |
|---|---|---|
| `ARRIVE_PX` | 32 | Waypoint reached within this; must exceed one step or it overshoots and spins back. |
| `TURN_TOL_DEG` | 12 | Only turn if heading error exceeds this (above turn residual). |
| `TURN_COMMIT_DEG` | 45 | After a turn, drive forward before turning again unless still badly off (anti-oscillation). |
| `TURN_SLOPE` / `TURN_COAST_DEG` | 1.05 / 8 | Turn-response compensation (see calibration). |
| `MAX_STEP_MM` | 50 | Max physical mm per forward step between observations. |
| `FORWARD_CMD_SCALE` | 3.2288 | Matches `main.py` FORWARD calibration. |
| `REPLAN_PX` | 400 | Only re-plan on MAJOR drift; low values churn (per-step correction handles normal drift). |
| `MAX_POSE_MISS` | 60 | Abort after this many consecutive frames with no marker. |

## Gotchas / operational notes

- **Camera env var**: set `os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"]="0"`
  **before** importing cv2, or camera init on Windows hangs. The controller does this;
  any ad-hoc test script must too.
- **Camera index 1** is correct here but is also used by the vision app — only one
  process can hold it. Close the other before running.
- **Stopping the robot reliably**: kill the **bridge** process on the robot by PID
  (`kill <pid>`); cutting the command stream halts the robot (it finishes its current
  small step and idles). Do **not** `pkill -f ev3_server.py` over SSH — the pattern
  matches the SSH shell's own command line and kills your session. Use an explicit PID
  or `fuser -k 9999/tcp`.
- **Executor startup**: `run_follow_loop` seeds `last_seq = -1`, so on startup it will
  execute whatever command currently sits in `cl_cmd.txt` if its seq ≠ -1. It's
  harmless if the last command was `STOP`, but to be safe it should seed `last_seq`
  from the current `cl_cmd.txt` sequence instead. (Small improvement, not yet done.)
- **Robot git**: `main.py` was deployed once via `scp` (CRLF). The clean committed
  version is on `origin/closed-loop`; on the robot run
  `git fetch && git reset --hard origin/closed-loop` to get the LF version. Content is
  identical (`turn_speed = 45`).

## Marker / detection notes

- Dictionary: **DICT_4X4_1000**, marker **id 0** (low ids are identical across all
  4X4 dicts, so an old 4x4_50 marker also works).
- The marker MUST sit on a **white background** that extends ≥1 module beyond its black
  border. `generate_aruco_marker.py` builds this; if you print your own, keep the
  white margin.
- Heading convention matches the planner: 0° = +x (right), 90° = down, −90° = up.
  An upright-mounted marker reads −90°, which equals `INITIAL_HEADING_DEG`.
