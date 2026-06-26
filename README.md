# GolfBot 9000
CDIO, Group 24

## Commands
Deps:
```bash
pip install opencv-python numpy
```

Run:
```bash
ssh robot@192.168.2.13
cd ~/CDIO-Mindstorms/robot/
brickrun -r -- pybricks-micropython main.py
```

Profiles:
```bash
python3 loop.py --profile home --camera 1
python3 loop.py --profile school --camera 1
```

## File Tree

```
CDIO-Mindstorms/
│
├── host/                    runs on host PC only, never deployed to EV3!!!
│   ├── loop.py                 main entry point
│   ├── detection.py            ball + ArUco detection (Roboflow YOLO)
│   ├── pathfinding.py          A* algo
│   ├── nav.py                  waypoint follower
│   ├── link.py                 Bluetooth / TCP linkage
│   ├── calibration.py          manual HSV calibration
│   ├── calibration_auto.py     automatic HSV calibration
│   ├── generate_aruco.py       tool, generate a robot ArUco marker
│   ├── config.py               shared constants
│   └── color_ranges.json       generated HSV constants
│
├── robot/                    deployed to EV3 (must be flat!)
│   ├── main.py                 PyBricks entry point, handles commands
│   ├── ev3_server.py           TCP bridge
│   ├── restart.sh              called over SSH by loop.py to restart the robot
│   └── commands.txt            active mission file
│
├── .env                      robot IP profiles + API keys (gitignored)
├── .env.example              template for .env
├── requirements.txt          python imports
└── README.md                 you are here!
```

## Hardware

| Component | Port | Purpose |
|---|---|---|
| Large Motor (Left wheel) | A | Drive |
| Large Motor (Right wheel) | B | Drive |
| Medium Motor (Lift) | C | Ball collection lift |
| Large Motor (Gate) | D | Ball delivery gate |
| Gyro Sensor | S4 | Rotation tracking (optional) |

Logitech USB camera on host PC for ball detection.

## Commands

| Command | Format | Description |
|---|---|---|
| `FORWARD` | `FORWARD:500` | Move forward N mm |
| `REVERSE` | `REVERSE:300` | Move backward N mm |
| `TURN` | `TURN:90` | Turn right N deg (negative = left) |
| `SPEED` | `SPEED:200` | Set speed in mm/s |
| `STOP` | `STOP` | Stop all movement |
| `LIFT_UP` | `LIFT_UP:180` | Raise collection lift N deg |
| `LIFT_DOWN` | `LIFT_DOWN:180` | Lower collection lift N deg |
| `GATE_OPEN` | `GATE_OPEN:90` | Open delivery gate N deg |
| `GATE_CLOSE` | `GATE_CLOSE:20` | Close delivery gate N deg |
| `SHAKE_LIFT` | `SHAKE_LIFT` | Shake lift to settle balls |
| `SAY` | `SAY:hello` | EV3 text-to-speech |
| `#` | `# comment` | Comment, ignored by controller |

## Team

Group 24: 
- Christian Hyllested
- Akkash Vigneswaran
- Hannah Enssafi Lund
- Leon Lorenzen
- Lilian Umar Osman
- Philip Crane

## Competition
Fully autonomous, 8-minute time limit, judge's decision is final.

| Event | Points |
|---|---|
| Ball delivered to Goal A | +150 |
| Ball delivered to Goal B | +100 |
| Orange VIP ball delivered first | +200 bonus |
| Per remaining second when all balls are collected | +3 |
| Robot touches course boundary or obstacles | -50 |
| Robot moves obstacles more than 1 cm | -100 |


Prioritize Goal A over B. Collect the orange VIP ball first. Finishing early earns bonus points, so route efficiency matters.
