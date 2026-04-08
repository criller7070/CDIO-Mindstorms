# LEGO Mindstorms EV3 - Complete Programming Guide

## Table of Contents
1. [Setup & Installation](#setup--installation)
2. [Hardware Components](#hardware-components)
3. [PyBricks Library Commands](#pybricks-library-commands)
4. [Motor Control](#motor-control)
5. [Movement Commands (DriveBase)](#movement-commands-drivebase)
6. [Sound & Speaker Commands](#sound--speaker-commands)
7. [Sensor Commands](#sensor-commands)
8. [Bluetooth Communication](#bluetooth-communication)
9. [Example Programs](#example-programs)
10. [Common Issues & Troubleshooting](#common-issues--troubleshooting)

---

## Setup & Installation

### Requirements
- LEGO Mindstorms EV3 Unit
- USB or Micro-USB Cable
- PyBricks MicroPython Firmware
- Python 3.7+

### Installing PyBricks Firmware
1. Download from: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3
2. Connect EV3 via USB
3. Follow the LEGO education website installation steps
4. Verify installation by running a simple beep command

### Installing PyBricks Library (for host machine)
```bash
pip install pybricks
```

---

## Hardware Components

### Standard EV3 Components
| Component | Port | Purpose |
|-----------|------|---------|
| Left Motor | Port A | Left wheel/movement |
| Right Motor | Port B | Right wheel/movement |
| Medium Motor | Port C | Additional actuator (optional) |
| Large Motor | Port D | Additional actuator (optional) |
| Touch Sensor | Port 1 | Detect physical pressure |
| Color Sensor | Port 2 | Detect colors/light |
| Ultrasonic Sensor | Port 3 | Distance detection |
| Gyro Sensor | Port 4 | Rotation/orientation detection |

### Port Mapping
```python
from pybricks.parameters import Port

Port.A    # Left Motor
Port.B    # Right Motor
Port.C    # Medium Motor
Port.D    # Large Motor
Port.S1   # Sensor Port 1
Port.S2   # Sensor Port 2
Port.S3   # Sensor Port 3
Port.S4   # Sensor Port 4
```

---

## PyBricks Library Commands

### Core Imports
```python
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
```

### EV3Brick - The Main Hub

#### Initialize the EV3
```python
ev3 = EV3Brick()
```

#### Speaker/Sound Methods
```python
ev3.speaker.set_volume(100)              # Set volume 0-100
ev3.speaker.beep(frequency=1000, duration=500)  # Beep with frequency (Hz) and duration (ms)
ev3.speaker.say("Hello World")           # Text-to-speech
ev3.speaker.play_file("/path/to/file.mp3")  # Play audio file
```

#### Light Methods
```python
ev3.light.on(Color.RED)                  # Turn on light with color
ev3.light.off()                          # Turn off light
ev3.light.blink(Color.ORANGE)            # Blink the light
```

#### Button Methods
```python
ev3.buttons.pressed()                    # Check which button is pressed
ev3.buttons.wait_for_press()             # Wait for any button press
ev3.buttons.wait_for_release()           # Wait for button release
```

#### Screen Methods
```python
ev3.screen.print("Message")              # Print message to screen
ev3.screen.clear()                       # Clear screen
```

---

## Motor Control

### Initialize Motors
```python
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Direction

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
medium_motor = Motor(Port.C)
large_motor = Motor(Port.D)
```

### Motor Methods

#### Basic Movement
```python
motor.run(500)                           # Run at constant speed (deg/s)
motor.run_time(500, 2000)                # Run for time (speed in deg/s, duration in ms)
motor.run_angle(500, 360)                # Run for angle (speed in deg/s, angle in degrees)
motor.run_target(500, 0)                 # Run to target position (speed, angle in degrees)
motor.stop()                             # Stop motor
motor.hold()                             # Hold position (active braking)
motor.reset_angle()                      # Reset angle to 0
```

#### Motor Properties
```python
motor.angle()                            # Get current angle position (degrees)
motor.speed()                            # Get current speed (deg/s)
```

#### Motor Settings
```python
motor.set_stop_action(Stop.COAST)        # Stop action: COAST, BRAKE, or HOLD
motor.control.limits(speed=500)          # Set speed limits
```

#### Motor Direction
```python
Motor(Port.A, direction=Direction.COUNTERCLOCKWISE)  # Reverse motor direction
```

---

## Movement Commands (DriveBase)

### Initialize DriveBase for Tank/Differential Drive
```python
from pybricks.robotics import DriveBase

robot = DriveBase(
    left_motor,           # Left motor
    right_motor,          # Right motor
    wheel_diameter=55.5,  # Wheel diameter in mm
    axle_track=104        # Distance between wheels in mm
)
```

### Drive Base Methods

#### Straight Movement
```python
robot.straight(500)                      # Move forward (mm) - positive = forward
robot.straight(-500)                     # Move backward (mm)
```

#### Turning
```python
robot.turn(90)                           # Turn clockwise (degrees)
robot.turn(-90)                          # Turn counterclockwise (degrees)
```

#### Curved Movement
```python
robot.curve(500, 100)                    # Move in curve (radius in mm, angle in degrees)
```

#### Continuous Movement
```python
robot.drive(200, 0)                      # Drive at speed and turn rate
robot.stop()                             # Stop all motors
```

#### Drive Base Settings
```python
robot.settings(
    straight_speed=500,                  # Default straight speed (mm/s)
    straight_acceleration=100,           # Acceleration for straight (mm/s²)
    turn_rate=90,                        # Default turn rate (deg/s)
    turn_acceleration=20                 # Acceleration for turns (deg/s²)
)
```

#### Drive Base Status
```python
robot.straight_speed                     # Get/set straight speed
robot.turn_rate                          # Get/set turn rate
robot.distance()                         # Get distance traveled since reset
robot.angle()                            # Get angle rotated since reset
robot.reset()                            # Reset distance and angle
```

---

## Sound & Speaker Commands

### Speaker Methods
```python
# Beep sound
ev3.speaker.beep()                       # Simple beep (default)
ev3.speaker.beep(frequency=1000)         # Beep at frequency (Hz)
ev3.speaker.beep(frequency=1000, duration=500)  # With duration (ms)

# Multiple beeps
ev3.speaker.beep()
ev3.speaker.beep()
ev3.speaker.beep()

# Speech synthesis
ev3.speaker.say("Hello")                 # Text-to-speech
ev3.speaker.say("Move forward")          # Phrase announcement

# Load and play sound files
ev3.speaker.play_file("path/to/sound.mp3")
ev3.speaker.play_file("path/to/sound.wav")

# Volume control
ev3.speaker.set_volume(100)              # Set volume 0-100
ev3.speaker.set_volume(50)               # 50% volume
ev3.speaker.set_volume(0)                # Mute
```

### Sound Examples
```python
# Startup sequence
ev3.speaker.beep()
ev3.speaker.say("Robot started")

# Movement announcements
ev3.speaker.say("Moving forward")
robot.straight(500)

# Success/failure sounds
ev3.speaker.beep(frequency=1000, duration=200)  # Success
ev3.speaker.beep(frequency=500, duration=300)   # Failure
```

---

## Sensor Commands

### Touch Sensor
```python
from pybricks.ev3devices import TouchSensor

touch = TouchSensor(Port.S1)

touch.pressed()                          # Boolean: is pressed?
touch.wait_until_pressed()               # Wait for press
touch.wait_until_released()              # Wait for release
```

### Color Sensor
```python
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Color

color_sensor = ColorSensor(Port.S2)

color_sensor.color()                     # Detected color (Color.RED, etc.)
color_sensor.reflection()                # Reflection value 0-100
color_sensor.rgb()                       # Raw RGB values (r, g, b)

# Color constants
Color.BLACK
Color.WHITE
Color.RED
Color.ORANGE
Color.YELLOW
Color.GREEN
Color.CYAN
Color.BLUE
Color.PURPLE
Color.BROWN
Color.GRAY
Color.NONE  # No color detected
```

### Ultrasonic Sensor (Distance)
```python
from pybricks.ev3devices import UltrasonicSensor

ultrasonic = UltrasonicSensor(Port.S3)

ultrasonic.distance()                    # Distance in mm
ultrasonic.presence()                    # Boolean: is object present?

# Example use
if ultrasonic.distance() < 200:          # Less than 200mm
    robot.stop()
    ev3.speaker.say("Object detected")
```

### Gyro Sensor
```python
from pybricks.ev3devices import GyroSensor

gyro = GyroSensor(Port.S4)

gyro.angle()                             # Current angle in degrees
gyro.speed()                             # Rotation speed in deg/s
gyro.reset_angle()                       # Reset angle to 0
```

---

## Bluetooth Communication

### Receiving Bluetooth Commands (On EV3)
```python
import bluetooth

class EV3SoundReceiver:
    def start_listening(self):
        """Listen for Bluetooth commands"""
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.bind(("", bluetooth.PORT_ANY))
        sock.listen(1)
        
        print("Listening for Bluetooth...")
        
        try:
            client_sock, client_info = sock.accept()
            print(f"Connected: {client_info}")
            
            while True:
                data = client_sock.recv(1024)
                if not data:
                    break
                
                command = data.decode('utf-8').strip()
                print(f"Command: {command}")
                self.execute_command(command)
        finally:
            client_sock.close()
            sock.close()
    
    def execute_command(self, command):
        """Execute received commands"""
        if command == "SOUND":
            ev3.speaker.beep()
        elif command == "MOVE":
            robot.straight(500)
        elif command == "TURN":
            robot.turn(90)
```

### Sending Bluetooth Commands (From Host Machine)
```python
import socket
import subprocess
import re

def find_ev3():
    """Find EV3 Bluetooth address"""
    try:
        result = subprocess.run(
            ["reg", "query", "HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Services\\BTHPORT\\Parameters\\Devices"],
            capture_output=True,
            text=True
        )
        addresses = re.findall(r'[0-9A-Fa-f]{12}', result.stdout)
        return [f'{addr[i:i+2]}' for i in range(0, 12, 2) for addr in addresses]
    except:
        return []

def send_command(address, command):
    """Send command to EV3"""
    try:
        sock = socket.socket(socket.AF_BTH, socket.SOCK_STREAM)
        sock.connect((address, 1))
        sock.send(command.encode())
        sock.close()
        print(f"Sent: {command}")
    except Exception as e:
        print(f"Error: {e}")

# Usage
addresses = find_ev3()
if addresses:
    send_command(addresses[0], "SOUND")
    send_command(addresses[0], "MOVE")
```

---

## Example Programs

### Example 1: Simple Movement
```python
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Move forward 1 meter
ev3.speaker.say("Moving forward")
robot.straight(1000)
ev3.speaker.beep()

# Move backward 1 meter
ev3.speaker.say("Moving backward")
robot.straight(-1000)
ev3.speaker.beep()

# Turn 360 degrees
ev3.speaker.say("Turning")
robot.turn(360)
ev3.speaker.beep()

# Done
ev3.speaker.say("Done")
```

### Example 2: Obstacle Avoidance with Ultrasonic Sensor
```python
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
ultrasonic = UltrasonicSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

ev3.speaker.say("Starting obstacle avoidance")

while True:
    distance = ultrasonic.distance()
    
    if distance > 300:  # No obstacle
        robot.drive(200, 0)
    elif distance > 150:  # Getting close
        ev3.speaker.beep(frequency=1000)
        robot.drive(50, 0)
    else:  # Too close
        ev3.speaker.say("Obstacle detected")
        robot.stop()
        robot.turn(90)
```

### Example 3: Line Following with Color Sensor
```python
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
color_sensor = ColorSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

ev3.speaker.say("Line following started")

while True:
    color = color_sensor.color()
    
    if color == Color.BLACK:
        # On black line
        robot.drive(200, 0)  # Go straight
    elif color == Color.WHITE:
        # Off line
        robot.drive(200, 30)  # Turn right to find line
```

### Example 4: Multi-Motor Control
```python
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

ev3 = EV3Brick()

# Initialize all motors
motor_a = Motor(Port.A)
motor_b = Motor(Port.B)
motor_c = Motor(Port.C)
motor_d = Motor(Port.D)

# Run different motors
motor_a.run_angle(500, 360)      # Full rotation
motor_b.run_time(500, 2000)      # Run for 2 seconds
motor_c.run(200)                 # Continuous rotation
motor_d.run_target(500, 180)     # Run to 180 degrees

# Wait for all to complete
ev3.speaker.say("All motors active")
```

### Example 5: Touch Sensor with Events
```python
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
touch = TouchSensor(Port.S1)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

ev3.speaker.say("Touch sensor ready")

while True:
    if touch.pressed():
        ev3.speaker.say("Button pressed")
        robot.stop()
        robot.turn(45)
    else:
        robot.drive(200, 0)
```

---

## Common Issues & Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'pybricks'"
**Solution:** Install PyBricks locally on your host machine
```bash
pip install pybricks
```

### Issue: Motor doesn't move
**Troubleshooting:**
- Check motor is connected to correct port
- Verify motor is not stalled (blocked)
- Test with simple `motor.run(500)` command
- Check battery level
- Ensure firmware is installed on EV3

### Issue: Sensor not detected
**Troubleshooting:**
- Verify sensor is connected to correct port
- Check sensor cable for damage
- Restart EV3
- Try sensor on different port
- Update firmware

### Issue: Bluetooth connection fails
**Troubleshooting:**
- Ensure EV3 Bluetooth is enabled
- Check EV3 address matches in code
- Pair devices first via OS settings
- Restart Bluetooth on both devices
- Check Windows firewall settings

### Issue: DriveBase movement is inaccurate
**Troubleshooting:**
- Calibrate wheel diameter (measure actual wheel)
- Check wheel alignment
- Verify motor speeds are balanced
- Adjust axle track measurement
- Level the surface

### Issue: Sound not working
**Troubleshooting:**
- Check volume is set above 0
- Verify speaker is not muted on EV3
- Test with `ev3.speaker.beep()`
- Restart EV3
- Check audio file format (.mp3, .wav)

---

## Quick Reference Cheat Sheet

### Essential Commands
```python
# Initialize
ev3 = EV3Brick()
robot = DriveBase(Motor(Port.A), Motor(Port.B), 55.5, 104)

# Movement
robot.straight(500)        # mm
robot.turn(90)             # degrees
robot.curve(500, 100)      # radius, angle
robot.drive(200, 0)        # speed, turn_rate
robot.stop()

# Sound
ev3.speaker.beep()
ev3.speaker.say("Text")

# Motors
motor.run(500)             # deg/s
motor.run_angle(500, 360)
motor.run_time(500, 2000)
motor.angle()

# Sensors
touch.pressed()
color_sensor.color()
ultrasonic.distance()
gyro.angle()
```

---

## Additional Resources

- **Official PyBricks Documentation**: https://pybricks.com/
- **LEGO Education Support**: https://education.lego.com/
- **EV3 Building Instructions**: https://education.lego.com/en-us/support/mindstorms-ev3/
- **PyBricks GitHub**: https://github.com/pybricks/pybricks-micropython

---

*Last Updated: April 2026*
*For LEGO Mindstorms EV3 with PyBricks MicroPython*
