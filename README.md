# GolfBot 9000 — Autonomous Golf Ball Collection Robot

**Course:** CDIO | **Group:** 24 | **Delivery:** March 26, 2026

---

## Competition

GolfBot competes in a pre-qualification round where groups demonstrate an autonomous robot that collects golf balls on a driving range. The best solution wins a prize and potential further collaboration.

The robot must collect as many balls as possible within 8 minutes on a delimited course and deliver them to one of two goals.

**Scoring**

| Event | Points |
|---|---|
| Ball delivered to Goal A | +150 |
| Ball delivered to Goal B | +100 |
| Orange VIP ball delivered first | +200 bonus |
| Per remaining second when all balls are collected | +3 |

**Penalties**

- −50 if robot touches course boundary or obstacles
- −100 if robot moves obstacles more than 1 cm

**Rules:** Fully autonomous, 8-minute time limit, judge's decision is final.

**Strategy implication:** Prioritize Goal A over B. Collect the orange VIP ball first. Finishing early earns bonus points, so route efficiency matters.

---

## Hardware

Platform: LEGO Mindstorms EV3 with PyBricks MicroPython firmware.

| Component | Port | Purpose |
|---|---|---|
| Large Motor (Left wheel) | A | Drive |
| Large Motor (Right wheel) | B | Drive |
| Medium Motor (Lift) | C | Ball collection lift |
| Large Motor (Gate) | D | Ball delivery gate |
| Gyro Sensor | S4 | Rotation tracking (optional) |

Optional: Logitech USB camera on host PC for vision-based ball detection.

---

## Architecture

**Primary — file-based (currently active):** The robot reads `commands.txt` at startup and executes the sequence fully autonomously. No Bluetooth required at runtime.

**Optional — Bluetooth sender:** `tools/tools_mission_sender.py` on the host PC can push a mission over Bluetooth. Windows-only via native AF_BTH socket (no PyBluez needed).

**Not currently active — vision-based:** `vision_boundary_detection.py` + `path_planner.py` provide an OpenCV-based alternative that detects balls and plans routes dynamically. Framework exists but integration is incomplete.

---

## Setup

### 1. Install host PC dependencies

```bash
pip install opencv-python numpy
```

### 2. Flash PyBricks firmware to EV3

Download from the LEGO Education website and follow their instructions. Verify it works with a test beep.

### 3. Transfer code to EV3

Copy `robot/main.py` and `robot/commands.txt` to the EV3 project folder. `main.py` is already named correctly for PyBricks.

Windows (robot-only deploy):

```powershell
powershell -ExecutionPolicy Bypass -File tools/deploy_robot_only.ps1
```

This uploads files from `robot/` only, never the full repository.

### 4. Pair EV3 via Bluetooth (if using host sender)

EV3: Settings → Bluetooth → Enable. Pair from host PC (PIN: 1234).

### 5. Design a mission

Edit `commands.txt` with the planned route:

```
SPEED:200
FORWARD:2000
TURN:90
GATE_OPEN:90
LIFT_UP:180
SHAKE_LIFT
GATE_CLOSE:20
STOP
```

### 6. Run

Power on the EV3 and run `main.py`. It loads `commands.txt` and executes automatically.

Or from host PC:
```bash
python tools/tools_mission_sender.py
```

---

## Commands

| Command | Format | Description |
|---|---|---|
| `FORWARD` | `FORWARD:500` | Move forward N mm |
| `REVERSE` | `REVERSE:300` | Move backward N mm |
| `TURN` | `TURN:90` | Turn right N° (negative = left) |
| `SPEED` | `SPEED:200` | Set speed in mm/s |
| `STOP` | `STOP` | Stop all movement |
| `LIFT_UP` | `LIFT_UP:180` | Raise collection lift N° |
| `LIFT_DOWN` | `LIFT_DOWN:180` | Lower collection lift N° |
| `GATE_OPEN` | `GATE_OPEN:90` | Open delivery gate N° |
| `GATE_CLOSE` | `GATE_CLOSE:20` | Close delivery gate N° |
| `SHAKE_LIFT` | `SHAKE_LIFT` | Shake lift to settle balls |
| `SAY` | `SAY:hello` | EV3 text-to-speech |
| `#` | `# comment` | Comment, ignored by controller |

Robot calibration values in `robot/main.py`:

```python
wheel_diameter = 55    # mm — measure your actual wheel
axle_track = 104       # mm — distance between wheel centers
```

---

## Troubleshooting

**Robot not moving** — check motors on ports A/B, verify battery level, recalibrate `wheel_diameter` and `axle_track`.

**Inaccurate movement** — measure actual wheel circumference, run on a flat surface, check for wheel slip. Use the gyro sensor for angle accuracy.

**Bluetooth not connecting** — ensure devices are paired, restart both, check EV3 MAC address in `tools/tools_mission_sender.py`.

**commands.txt not found** — the robot searches `/home/robot/`, `/media/`, and the project root. Place it in the project root when deploying.

**Wrong camera index (vision)** — run `test_camera.py` to scan for available cameras and find the correct index for your Logitech device.

---

## Team

Group 24: Christian Hyllested, Akkash Vigneswaran, Hannah Enssafi Lund, Leon Lorenzen, Lilian Umar Osman, Philipp Zhuravlev.
