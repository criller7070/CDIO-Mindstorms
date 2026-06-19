# Contributing & Project Structure

## Quick Start (open-loop mission)
```
ssh robot@10.56.138.36
cd ~/CDIO-Mindstorms/robot/
brickrun -r -- pybricks-micropython main.py
```

## TODO
- Pathfinding currently treats the robot as a point, but it has real dimensions (width, length, and the doors/gates sticking out at certain angles). This means paths that look clear on paper can clip the wall, the cross, or other obstacles in practice, especially in tight corners or near the dropoff hole. The pathfinder needs to account for the robot's full footprint (and gate state, since open gates change the effective footprint) when checking for collisions, not just the centerpoint. Ideas to explore: inflate obstacles by half the robot's width/length as a simple first pass, or model the robot as a proper polygon/rectangle for collision checks if more precision is needed.

- The doors are currently open the entire time the robot is moving, but they should only open right before a ball pickup. Keeping them open in transit causes two problems: balls already in the tray can fall out while driving, and the open doors can collide with the wall or other obstacles in narrow spaces. Doors should default to closed during travel and only open in the brief window right before/during a pickup maneuver, then close again immediately after. Need to define what "right before pickup" means precisely (e.g. triggered by proximity to the ball / arrival at the pickup waypoint) so the timing isn't too early or too late.

- Dropoff logic: The robot needs to enter the hole perpendicularly to the wall with the nose facing the wall, with the gates slightly open (45 deg). This means we have to make sure that the robot is ALREADY perpendicular before it reaches the wall rather than correcting the last second. Currently the alignment correction happens too close to the wall, so there's no room left to actually fix the angle, which risks bumping the wall or missing the hole. The robot should be heading-aligned well before the final approach segment so the last stretch into the hole is a straight, already-perpendicular line. This is the same underlying issue as the waypoint heading-correction problem below, so fixing one likely helps the other — heading needs to be reached progressively during approach, not patched at the end.

- Dropoff part 2: The tray needs to be lifted when dropoff has been [triggered/completed]. The lift should be about 45 degrees, just to tip the tray but not enough to lift it. The 45-degree tip should be enough to let balls roll out into the hole under gravity without the tray going so high that the tray tips on its side. Need to define the trigger condition for when the lift happens (e.g. once the robot has confirmed it's stopped inside/at the hole, perpendicular and gates positioned correctly), and make sure the lift motion itself doesn't push the robot off its aligned position.

- The robot should not touch neither the wall nor the cross in the middle. This is a general safety/collision constraint that ties into the pathfinding footprint issue above — both the wall boundary and the center cross need to be treated as hard obstacles with some margin, not just lines to path around exactly. Worth checking whether current near-wall maneuvers (like the dropoff approach and wall-adjacent ball pickups) leave any margin at all, or whether they rely on the robot stopping exactly in time.

- Balls near the wall need to be accessed the same way as the drop-off logic, meaning perpendicular to the wall, with the robot facing the wall. Right now wall-adjacent pickups likely use the same generic approach logic as open-field pickups, which doesn't account for the wall being there. These pickups need the same "already perpendicular before arrival" behavior as the dropoff fix, plus probably the same kind of footprint/clearance awareness so the robot doesn't clip the wall while turning in to grab the ball.

- Show which balls are detected on closed-loop control screen. Right now ball detections aren't visible during operation, which makes it hard to debug pickup misses or verify the vision system is seeing what we expect in real time, whether its false positives or missed balls. Add an overlay or marker on the control screen showing each currently detected ball's position (and ideally whether it's currently targeted for pickup), so issues with detection vs. actual robot behavior can be diagnosed live instead of after the fact. Bonus points if the marker also features a number relative to which order the balls are picked up

- Waypoint heading correction needs to be smarter. Right now a waypoint just stores a target heading, and the robot drives to the checkpoint position first, then checks afterward whether its heading is off and corrects it as a separate step. This reactive, post-arrival correction is unreliable and sometimes leaves the robot not facing the correct direction. Heading needs to be reached *during* the approach, not fixed up after the fact — same root issue as the dropoff perpendicularity problem above, so a fix here likely helps that too. Ideas to explore: interpolate heading along the final approach segment so the robot is already rotating into position as it arrives, rather than treating position-arrival and heading-correction as two sequential steps; consider separate tolerances for position vs. heading so arrival can require both to be satisfied at once instead of triggering a late correction; check whether current tolerance/threshold values are causing correction to kick in too late or stop too early.

## Future Work
- The tray can hold 8 balls, so the max capacity should be corrected. Whatever value is currently configured as max tray capacity doesn't match the physical tray, which could cause the robot to either stop picking up balls too early (leaving capacity unused) or assume it has room when it doesn't. Find wherever capacity is defined/used in pickup decision logic and update it to 8, and double check nothing else (like weight/balance assumptions) depends on the old number.

- Balls in corners are difficult to reach; the gates need to be slightly open (45 degrees) to grab them. Fully closed gates can't get around a ball sitting tight in a corner, but fully open gates may be too wide for the confined space. A 45-degree partial opening is likely the sweet spot — enough clearance to scoop the ball without the gates colliding with both walls of the corner at once. Needs testing to confirm 45 degrees is actually the right angle across different corner geometries, and logic to detect "this is a corner pickup" so the robot knows to use the partial-open behavior instead of the normal pickup gate angle.


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
