# Contributing & Project Structure

## Proposed Directory Layout

The codebase is currently flat. This is the proposed reorganization:

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

## PyBricks Flat Structure Constraint

PyBricks officially only supports flat project structures. No subdirectories inside `robot/`. All files in the `robot/` folder get copied to the EV3 as-is when deployed via the LEGO Education IDE or pybricksdev.

Subdirectory imports are undocumented and unreliable — keep `robot/` flat.

## Switching Missions

To run a different mission, copy from `missions/` into `robot/`:

```bash
# Windows
copy missions\commands_gate_test.txt robot\commands.txt

# Mac/Linux
cp missions/commands_gate_test.txt robot/commands.txt
```

Then redeploy to the EV3.

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

## Branches

Work on feature branches. `main` should always have a working mission file and robot code that can be deployed to the EV3.
