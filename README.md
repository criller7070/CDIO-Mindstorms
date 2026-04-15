# 🤖 GolfBot 9000: Autonomous Golf Ball Collection Robot

**Project Name (Code):** ClankerButt3000  
**Course:** CDIO Project  
**Group:** 24  
**Delivery:** March 26, 2026  
**Current Approach:** Pre-Planned Route Navigation

---

## 📋 Table of Contents

1. [Vision & Overview](#vision--overview)
2. [Project Objectives](#project-objectives)
3. [Navigation Approach](#navigation-approach)
4. [Hardware Components](#hardware-components)
5. [Software Architecture](#software-architecture)
6. [Setup & Installation](#setup--installation)
7. [Usage Guide](#usage-guide)
8. [Project Structure](#project-structure)
9. [Key Features](#key-features)
10. [Risk Management](#risk-management)
11. [Ethical Considerations](#ethical-considerations)
12. [Team & Timeline](#team--timeline)

---

## 🎯 Vision & Overview

**Vision Statement:**
> For golfers who value a clean and well-maintained golf course, GolfBot 9000 is an autonomous robot that collects stray golf balls around the course. Unlike manual assistants who must perform the work themselves, GolfBot operates efficiently and independently, keeping the course tidy with minimal human effort.

**Purpose:** This project develops an autonomous robot built on LEGO Mindstorms EV3 that can:
- Navigate a predefined path on a golf course or controlled environment
- Detect and collect white and orange golf balls (simulated with table tennis balls)
- Autonomously follow a pre-planned mission sequence
- Execute precise movement commands with accurate distance and angle measurements
- Report status and receive commands via Bluetooth

> **📌 Current Implementation:** The robot uses **pre-planned route navigation** via command sequences. The vision-based system is available as an alternative approach for future development.

---

## 🎯 Project Objectives

The robot must achieve the following milestones:

| Week | Milestone | Status |
|------|-----------|--------|
| 4 | Design finalized; key components (sensors/motors) selected | ✓ |
| 8 | First functional prototype; robot can move independently | ✓ |
| 12 | All core systems integrated; autonomous ball collection following planned route | In Progress |
| 16 | Full system operational; stable demonstration ready | Planned |

---

## 🗺️ Navigation Approach

### **PRIMARY: Pre-Planned Route Navigation** ✅ (Currently Active)

The robot follows a **predefined sequence of movements** that are carefully planned for a specific environment:

```
┌─────────────────────────────────────────────┐
│    PLANNED MISSION SEQUENCE                 │
│    (Optimized for target environment)       │
│                                             │
│    FORWARD:2000mm                          │
│    TURN:90° RIGHT                          │
│    FORWARD:1500mm                          │
│    TURN:45° LEFT                           │
│    FORWARD:800mm                           │
│    [Ball Collection at each stop point]    │
│    ...                                      │
└────────────┬────────────────────────────────┘
             │ (commands.txt)
             ▼
┌─────────────────────────────────────────────┐
│    HOST PC (Python)                         │
│    • Loads mission file                     │
│    • Sends commands to EV3                  │
│    • Monitors robot execution               │
│    • Handles Bluetooth communication        │
└────────────┬────────────────────────────────┘
             │ Bluetooth
             ▼
┌─────────────────────────────────────────────┐
│    EV3 ROBOT (PyBricks)                     │
│    • Receives command sequence              │
│    • Executes each movement precisely       │
│    • Activates collection mechanism         │
│    • Reports completion status              │
└─────────────────────────────────────────────┘
```

**Advantages:**
- ✅ Highly predictable and repeatable
- ✅ Optimized for specific environment/layout
- ✅ No real-time processing overhead
- ✅ Fewer dependencies (no camera calibration needed)
- ✅ Reliable performance in controlled settings

**How It Works:**
1. **Plan Phase:** Route is designed with precise distances and angles
2. **Command Generation:** Sequence exported to `commands.txt`
3. **Execution:** EV3 reads commands sequentially and executes each movement
4. **Collection:** Robot collects balls at predefined waypoints
5. **Verification:** Host PC monitors progress and confirms completion

### **ALTERNATIVE: Vision-Based Navigation** (Not Currently Active)

The robot uses **computer vision** to dynamically navigate and find balls in real-time:

```
Logitech Camera → Ball Detection → Path Planning → Navigation
```

**Status:** Available for future enhancement or alternative testing  
**See:** [VISION_BOUNDARY_GUIDE.md](VISION_BOUNDARY_GUIDE.md) for setup instructions

---

## ⚙️ Hardware Components

### EV3 Brick & Motors
| Component | Port | Purpose |
|-----------|------|---------|
| **Large Motor (Left)** | Port A | Left wheel drive |
| **Large Motor (Right)** | Port B | Right wheel drive |
| **Medium Motor (Optional)** | Port C | Ball collection mechanism |
| **EV3 Brick** | Main Hub | Robot controller & processor |

### Sensors
| Sensor | Port | Purpose |
|--------|------|---------|
| **Touch Sensor (Optional)** | Port 1 | Obstacle detection / collection trigger |
| **Color Sensor (Optional)** | Port 2 | Optional for line/boundary following |
| **Ultrasonic Sensor** | Port 3 | Distance measurement & obstacle avoidance |
| **Gyro Sensor** | Port 4 | Rotation tracking & accurate turning |

### Communication
- EV3 internal battery
- **Bluetooth connectivity** between EV3 and host PC
- USB Micro-B charging cable

### Optional Vision System
- **Logitech USB Camera** (for vision-based alternative approach)
- Streams real-time video for dynamic ball detection
- Communicates steering commands to EV3 via Bluetooth

---

## 🏗️ Software Architecture

### **PRIMARY WORKFLOW: Pre-Planned Route**

```
┌──────────────────────────────────────────────────────┐
│         MISSION PLANNING STAGE                       │
│    Design optimal route with waypoints              │
│    Calculate distances and turn angles              │
│    Export to commands.txt                           │
└──────────────────────┬───────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────┐
│         HOST PC (Python) - Execution Manager         │
│    main.py (or ev3_nav_controller.py)               │
│    • Reads commands.txt                             │
│    • Establishes Bluetooth connection               │
│    • Sends commands sequentially                    │
│    • Monitors robot status                          │
│    • Logs execution results                         │
└──────────────────────┬───────────────────────────────┘
                       │ Bluetooth
                       ▼
┌──────────────────────────────────────────────────────┐
│    EV3 ROBOT - Mission Executor                     │
│    ev3_nav_controller.py (main.py)                  │
│    • Initializes motors & sensors                   │
│    • Receives command queue                         │
│    • Executes: FORWARD, TURN, REVERSE, SPEED, STOP │
│    • Coordinates collection mechanism               │
│    • Reports completion & errors                    │
└──────────────────────────────────────────────────────┘
```

### **ALTERNATIVE WORKFLOW: Vision-Based** (Optional)

```
┌─────────────────────────────────────────────┐
│         HOST PC (Python)                    │
│  vision_boundary_detection.py               │
│  • Captures video from Logitech camera      │
│  • Detects balls (white & orange)           │
│  • Detects obstacles/boundaries             │
│  • Generates autonomous path in real-time   │
│  • Sends commands via Bluetooth             │
└────────────┬────────────────────────────────┘
             │ Bluetooth
             ▼
┌─────────────────────────────────────────────┐
│         EV3 ROBOT (PyBricks)                │
│  Executes dynamic movement commands         │
└─────────────────────────────────────────────┘
```

---

### Core Modules

#### **1. ev3_nav_controller.py** (EV3-side) ⭐ PRIMARY
Autonomous path execution engine for the EV3 brick:
- Initializes motors and drive base
- Parses **pre-planned command sequences** (FORWARD, TURN, REVERSE, SPEED, STOP)
- Executes each movement with precise distance/angle control
- Activates collection mechanism at waypoints
- Reports completion and status

#### **2. vision_boundary_detection.py** (Host PC) 📷 OPTIONAL
Computer vision & dynamic path planning (available for alternative approach):
- **BallHuntingPlanner class**: Manages ball detection and pathfinding
- Detects white balls: HSV range [0, 0, 200] - [180, 30, 255]
- Detects orange balls: HSV range [5, 100, 100] - [25, 255, 255]
- Generates autonomous traversal commands in real-time
- Writes mission commands to `commands.txt`

#### **3. ev3_receiver.py** (EV3-side)
Bluetooth communication handler:
- Listens for incoming commands from host PC
- Decodes command packets
- Passes instructions to motor controller

#### **4. ev3_receiver_sound.py** (EV3-side)
Sound & feedback system:
- Provides audio feedback for robot status
- Beeps for successful commands
- Text-to-speech for informational messages

#### **5. Test Modules**
- `test_camera.py` - Verify Logitech camera connectivity (for vision approach)
- `test_sound.py` - Test EV3 speaker functionality
- `test.py` - General system testing

---

## 🛠️ Setup & Installation

### Prerequisites
- Python 3.7 or higher
- LEGO Mindstorms EV3 with PyBricks firmware
- Windows, macOS, or Linux host PC
- Bluetooth capability on host PC
- *Optional:* Logitech USB camera (for vision-based approach)

### Step 1: Install Host PC Dependencies

```bash
# Clone or navigate to the project directory
cd CDIO-Mindstorms

# Install required Python packages
pip install opencv-python      # Computer vision (optional, for vision approach)
pip install numpy              # Numerical operations
pip install pybricks           # PyBricks library (optional for host)
```

> **Note:** Windows users don't need PyBluez; the system uses native Windows Bluetooth sockets (AF_BTH).

### Step 2: Set Up EV3 Robot

1. **Update PyBricks firmware:**
   - Download from: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3
   - Follow LEGO's installation instructions
   - Verify with a test beep command

2. **Transfer code to EV3:**
   - Copy `ev3_nav_controller.py` to the EV3 and rename to `main.py`
   - Or run via IDE: LEGO Education App or PyCharm

3. **Enable Bluetooth on both devices:**
   - **EV3:** Settings → Bluetooth → Enable
   - **Host PC:** Settings → Bluetooth & devices → Turn on
   - **Pair devices:** Select EV3 from available devices (PIN: 1234)

4. **Test EV3 connectivity:**
   ```bash
   python test_sound.py  # Should trigger beeps and speech
   ```

### Step 3: Prepare Mission File

Create or design your `commands.txt` file with the planned route:

```
# GolfBot Mission Plan
SPEED:200
FORWARD:2000
TURN:90
FORWARD:1500
TURN:-45
FORWARD:800
SPEED:150
FORWARD:500
REVERSE:300
STOP
```

Parameters:
- `FORWARD:distance` - Move forward (mm)
- `TURN:angle` - Turn right (positive) or left (negative) in degrees
- `REVERSE:distance` - Move backward (mm)
- `SPEED:value` - Set movement speed (mm/s)
- `STOP` - Emergency stop

### Step 4: Run the System

#### On EV3:
```
Run main.py or ev3_nav_controller.py
Expected output: "Navigation ready" → "Waiting for connection..."
```

#### On Host PC:
```bash
# Execute the pre-planned mission
python ev3_nav_controller.py

# Or use Bluetooth sender:
python -c "
import socket
sock = socket.socket(socket.AF_BTH, socket.SOCK_STREAM)
sock.connect(('[EV3_MAC_ADDRESS]', 1))
sock.send(b'FORWARD:2000\n')
"
```

The system will:
1. Load mission sequence from `commands.txt`
2. Establish Bluetooth connection to EV3
3. Send commands sequentially
4. Monitor command execution and status
5. Report completion or errors

---

### Optional: Camera Setup (for Vision-Based Alternative)

```bash
# Test camera connectivity
python test_camera.py
```

If the camera isn't detected at index 0, update the camera index in `vision_boundary_detection.py`:
```python
self.cap = cv2.VideoCapture(1)  # Change index if needed
```

---

## 📁 Project Structure

```
CDIO-Mindstorms/
├── README.md                           # This file
├── EV3_MINDSTORMS_COMMANDS.md         # EV3 command reference
├── VISION_BOUNDARY_GUIDE.md           # Vision system setup guide (optional)
├── commands.txt                        # ⭐ MISSION COMMAND SEQUENCE (PRIMARY)
├── gyal.mp3                           # Audio file for EV3
│
├── EV3 Controller Modules (run on robot):
│   ├── ev3_nav_controller.py          # ⭐ Main navigation controller (PRIMARY)
│   ├── ev3_receiver.py                # Bluetooth receiver
│   └── ev3_receiver_sound.py          # Sound & audio feedback
│
├── Vision-Based Navigation (optional - currently not active):
│   ├── vision_boundary_detection.py   # Ball detection & pathfinding
│   └── commands.txt                   # Generated mission commands
│
├── Testing Modules:
│   ├── test.py                        # General system tests
│   ├── test_camera.py                 # Camera connectivity test (optional)
│   └── test_sound.py                  # EV3 speaker test
│
└── robot_educator_basic/
    └── main.py                        # Example LEGO Educator program
```

---

## 🚀 Usage Guide

### Basic Workflow - Pre-Planned Route ⭐

**1. Design Your Mission:**
Create or edit `commands.txt` with precise waypoints:
```
SPEED:200
FORWARD:2000       # Move 2 meters forward
TURN:90            # Turn right 90°
FORWARD:1500       # Move 1.5 meters forward
TURN:-45           # Turn left 45°
FORWARD:800        # Approach ball collection point
REVERSE:300        # Back up
STOP
```

**2. Start the EV3 Robot:**
- Power on the EV3
- Run `main.py` or deploy via IDE
- Wait for "Waiting for connection..." message

**3. Execute Mission (Host PC):**
```bash
python ev3_nav_controller.py
```

**4. Monitor Execution:**
- Console shows each command being sent
- EV3 displays command status on screen
- System confirms when each movement completes
- Robot activates collection mechanism at waypoints

**5. Review Results:**
- Check console output for total time and success/failure
- Verify all waypoints were reached
- Confirm balls collected at each location

### Command Format Reference

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Move Forward | `FORWARD:distance` | `FORWARD:500` | Move forward 500mm |
| Turn | `TURN:angle` | `TURN:90` | Turn right 90°, or `TURN:-45` for left |
| Move Backward | `REVERSE:distance` | `REVERSE:300` | Move backward 300mm |
| Set Speed | `SPEED:value` | `SPEED:250` | Set speed to 250 mm/s |
| Stop | `STOP` | `STOP` | Emergency stop |
| Comment | `# text` | `# First approach` | Ignored by controller |

### Robot Parameters

Update these values in `ev3_nav_controller.py` to match your physical robot:

```python
wheel_diameter = 55.5        # Millimeters
axle_track = 104             # Distance between wheels (mm)
forward_speed = 200          # mm/s
turn_speed = 90              # degrees/s
```

---

## ✨ Key Features

### ✅ **Pre-Planned Route Navigation** (Primary)
- Highly reliable and repeatable performance
- Optimized paths for specific environments
- Precise distance and angle control
- No real-time processing required
- Minimal sensor dependencies

### 🎥 **Vision-Based Navigation** (Optional Alternative)
- Real-time ball detection (white and orange)
- Automatic boundary and obstacle recognition
- Dynamic path planning to adapt to environment
- Autonomous decision-making
- Greater flexibility for varied layouts
- *Currently not active - available for future use*

### 🤖 **Hardware Integration**
- DriveBase control for smooth, synchronized movement
- Motor synchronization for straight lines and accurate turns
- Sensor fusion for optional obstacle avoidance
- Bluetooth for remote monitoring and control

### 🔄 **Modular Design**
- Separate host PC and EV3 processes
- Clean command interface (`commands.txt`)
- Easy to extend with new behaviors
- Comprehensive logging and debugging

### 📊 **Real-Time Monitoring**
- Console status updates during mission
- Robot position and completion tracking
- Command execution feedback
- Error reporting and recovery

---

## ⚠️ Risk Management

### Identified Risks & Mitigation Strategies

#### 1. **Motor Accuracy Issues** (Probability: 2, Impact: 3)
**Risk:** Robot doesn't move precise distances or angles
**Mitigation:**
- Calibrate wheel diameter and axle track measurements
- Test on flat, level surfaces
- Build in correction commands if drift detected
- Implement sensor feedback (gyro) for angle verification

#### 2. **Bluetooth Connection Drops** (Probability: 2, Impact: 3)
**Risk:** Communication lost during mission execution
**Mitigation:**
- Use robust Bluetooth pairing
- Implement automatic reconnection
- Add watchdog timeout for hung commands
- Log all communication for debugging

#### 3. **Route Plan Becomes Invalid** (Probability: 1, Impact: 2)
**Risk:** Environment changes or obstacles block planned path
**Mitigation:**
- Verify environment layout before mission
- Add safety margins around waypoints
- Implement obstacle avoidance checks
- Have fallback routes prepared

#### 4. **Timeline Slippage** (Probability: 1, Impact: 3)
**Risk:** Project falls behind schedule
**Mitigation:**
- Pre-planned route is faster to implement than vision system
- Prioritize core movement functionality
- Track progress weekly via Gantt chart
- Flexible reallocation of tasks as needed

#### 5. **Collection Mechanism Failure** (Probability: 2, Impact: 3)
**Risk:** Ball collection mechanism jams or doesn't work
**Mitigation:**
- Test collection mechanics separately before integration
- Design for quick mechanical repairs or replacement
- Build redundancy into collection actuators

**Risk Matrix:** (Likelihood × Impact)
- Low: 1-2 (Green)
- Medium: 3-6 (Yellow)
- High: 6-9 (Red)

---

## 🧠 Ethical Considerations

### Program Scope & Transparency
While the pre-planned route approach is more deterministic, the system includes optional vision capabilities that use cameras. Ethical considerations:

**Privacy & Surveillance**
- The camera system (if activated) may incidentally capture people
- **Solution:** Process only ball-detection data; no storage of video frames

**Employment Impact**
- Automation may affect greenkeeper and maintenance staff jobs
- **Balanced approach:**
  - Robot complements rather than replaces workers
  - Frees staff for higher-level maintenance tasks
  - Improves course upkeep efficiency

### Responsible Development
As engineers, we commit to:
- ✅ Operating robot safely in controlled environments
- ✅ Using image recognition (if activated) only for ball detection
- ✅ Avoiding unnecessary data storage or privacy violations
- ✅ Ensuring data is processed locally and deleted after use
- ✅ Informing users if cameras are in use
- ✅ Minimizing negative social consequences
- ✅ Maintaining technical robustness and safety

---

## 👥 Team & Timeline

### Team Members (Group 24)

| Name | Role | Phase Lead | Responsibilities |
|------|------|-----------|------------------|
| Christian Hyllested | Project Lead | Phase 1 | Overall coordination |
| Akkash Vigneswaran | Lead Researcher | Phase 1 | Component research |
| Hannah Enssafi Lund | Lead Developer | Phase 2 | Prototype development |
| Leon Lorenzen | Integration Lead | Phase 3 | System integration |
| Lilian Umar Osman | Test Lead | Phase 4 | Testing & validation |
| Philipp Zhuravlev | Developer | All | Development support |

### Project Timeline (16 Weeks)

**Phase 1: Research & Design (Weeks 1-4)** ✓ Complete
- Equipment evaluation and sensor testing
- Basic movement & steering implementation
- System architecture design
- **Phase Lead:** Akkash

**Phase 2: First Prototype (Weeks 5-8)** ✓ Complete
- Build chassis and basic structure
- Implement navigation
- Develop command execution system
- Test individual components
- **Phase Lead:** Hannah

**Phase 3: Integration & Improvement (Weeks 9-12)** 🔄 Current
- Finalize pre-planned route navigation
- Develop and test collection mechanism
- System stability and debugging
- Full integration testing
- **Phase Lead:** Leon

**Phase 4: Testing & Demonstration (Weeks 13-16)** 📅 Upcoming
- Comprehensive system testing
- Performance optimization
- Prepare for final demo
- Documentation finalization
- **Phase Lead:** Lilian

**Total Estimated Hours:** 1,400 hours (26 working days × 9 hours × 6 people)

---

## 📖 Documentation

### Guides Included
- **EV3_MINDSTORMS_COMMANDS.md** - Complete PyBricks command reference
- **VISION_BOUNDARY_GUIDE.md** - Camera setup and calibration guide (optional)
- **commands.txt** - Mission command format documentation

### Project Reports
See the PDF submission (Groupe24_aflevering_2.pdf) for:
- Updated project plan with progress tracking
- Revised risk analysis
- Project status report
- System architecture sketches

---

## 🐛 Troubleshooting

### Robot Not Moving
- Verify motors are connected to Ports A & B
- Check wheel alignment and friction
- Test motor speeds in `ev3_nav_controller.py`
- Calibrate wheel parameters (diameter, axle_track)
- Check battery level on EV3

### Bluetooth Connection Issues
- Ensure both devices are paired
- Check if Bluetooth is enabled on both
- Restart both devices and re-pair
- Windows: Run Bluetooth troubleshooter
- Check MAC address matches in connection code

### Commands Not Executing
- Verify `commands.txt` format (no extra spaces)
- Check EV3 is in "Waiting for connection..." state
- Ensure Bluetooth connection is active
- Review console output for error messages

### Inaccurate Movement
- Recalibrate wheel diameter (measure actual wheel circumference)
- Verify robot is on flat, level surface
- Check for wheel slippage or friction issues
- Adjust speed values if too fast/slow
- Use gyro sensor to verify angle accuracy

### Ball Detection Issues (Vision System)
- Run color calibration test
- Adjust HSV ranges for your lighting conditions
- Ensure balls are visible to the camera
- Check camera lens for dust/damage
- Verify camera index matches actual device

---

## 📞 Support & Contact

For issues, questions, or contributions:
- Refer to EV3_MINDSTORMS_COMMANDS.md for hardware commands
- Check VISION_BOUNDARY_GUIDE.md for vision setup (optional)
- Review project plan and risk analysis in submission PDF
- Contact project lead for additional support

---

**Project Status:** 🟡 In Progress (Week 9 of 16 - Phase 3)  
**Navigation Approach:** ⭐ Pre-Planned Route (Primary)  
**Alternative Available:** 📷 Vision-Based (Optional)  
**Last Updated:** April 15, 2026  
**Next Milestone:** Full system integration and testing (Week 12)
