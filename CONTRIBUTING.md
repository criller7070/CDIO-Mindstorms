# Contributing & Project Structure

## Quick Start (open-loop mission)
```
ssh robot@10.56.138.36
cd ~/CDIO-Mindstorms/robot/
brickrun -r -- pybricks-micropython main.py
```

## TODO
- Streamline different processes for
    - 1) pickup: gate/lift motor sequencing, ball scooping angle and timing
    - 2) nav: closed-loop follow loop — step sizing, turn compensation, arrival tolerance
    - 3) planning: A* routing, TSP ball ordering, multi-trip clustering
    - 4) detection: ball/field/robot pose detection (vision_detector.py)
    - 5) calibration (colors): HSV ranges for WHITE/ORANGE/RED (color_ranges.json)
    - 6) calibration (movement): physical constants — FORWARD_CMD_SCALE, TURN_SLOPE, TURN_COAST_DEG
    - 7) EV3 firmware: command dispatcher, follow loop, gyro turns (robot/main.py)
    - 8) transport: TCP/Bluetooth bridge, file handshake, link classes (ev3_server.py)
    - 9) vision app / UI: live overlays, planning trigger, screenshots (vision_app.py)
    - 10) deployment / tooling: deploy scripts, ArUco generator, mission variants
- Make Bluetooth sender and deploy scripts Linux-compatible (tools_mission_sender.py uses Windows-only AF_BTH; deploy_robot_only.ps1 is PowerShell — vision UI already works on Linux with QT_QPA_PLATFORM=xcb)


## File Tree

```
CDIO-Mindstorms/
│
├── robot/                      ← deployed to EV3 (must stay FLAT — PyBricks constraint)
│   ├── main.py                 ← ev3_nav_controller.py renamed; PyBricks expects main.py
│   ├── ev3_receiver.py
│   ├── ev3_receiver_sound.py
│   ├── commands.txt            ← active mission file (must be here at deployment)
│   └── gyal.mp3
│
├── tools/                      ← runs on PC only, never deployed to EV3
│   ├── tools_mission_sender.py
│   ├── tools_vision_boundary_detection.py
│   ├── tools_path_planner.py
│   └── requirements.txt
│
├── missions/                   ← mission variants; copy chosen one → robot/commands.txt
│   ├── commands_gate_test.txt
│   ├── commands_gate_swapped_test.txt
│   └── commands_spin.txt
│
├── tests/
│   ├── test.py
│   ├── test_navigation.py
│   ├── test_camera.py
│   └── test_sound.py
│
├── docs/
│   ├── IMPLEMENTATION_PROGRESS.md
│   ├── EV3_MINDSTORMS_COMMANDS.md
│   └── VISION_BOUNDARY_GUIDE.md
│
└── README.md
```

## Switching Missions

To run a different mission, copy from `missions/` into `robot/`:

```bash
# Windows
copy missions\commands_gate_test.txt robot\commands.txt

# Mac/Linux
cp missions/commands_gate_test.txt robot/commands.txt
```

Then redeploy to the EV3.

## Deploying To EV3 (Robot-Only)

Use this from repository root on Windows:

```powershell
powershell -ExecutionPolicy Bypass -File tools/deploy_robot_only.ps1
```

This uploads files from `robot/` only (flat to `/home/robot/`) and avoids sending the full repo.

## Command Format

Mission files use plain text, one command per line:

```
# Lines starting with # are comments and are ignored
SPEED:200
FORWARD:500
TURN:90
GATE_OPEN:90
STOP
```

See `docs/EV3_MINDSTORMS_COMMANDS.md` for the full command reference.

## PC Tools Setup

```bash
cd tools
pip install -r requirements.txt
python tools_mission_sender.py
```

No extra Bluetooth library needed on Windows — uses native AF_BTH socket.
