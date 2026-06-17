#!/usr/bin/env pybricks-micropython

"""
EV3 Navigation Controller - Autonomous Path Execution
Reads commands from commands.txt and executes a full mission
Commands: FORWARD:distance, TURN:angle, REVERSE:distance, SPEED:value, STOP
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
import time
import threading

class EV3NavController:
    def __init__(self):
        """Initialize EV3 robot"""
        self.ev3 = EV3Brick()
        self.ev3.screen.clear()
        self.ev3.screen.print("Init...")
        
        # Queue for voice commands (run one at a time)
        self.voice_queue = []
        self.voice_speaking = False
        
        # Spinning control
        self.spinning = False
        self.spin_speed = 300
        
        # Test Port A
        try:
            self.left_motor = Motor(Port.A)
            print("[OK] Port A motor found")
        except Exception as e:
            self.ev3.screen.clear()
            self.ev3.screen.print("Port A Failed")
            print("[ERROR] Port A: {}".format(str(e)))
            raise
        
        # Test Port B
        try:
            self.right_motor = Motor(Port.B)
            print("[OK] Port B motor found")
        except Exception as e:
            self.ev3.screen.clear()
            self.ev3.screen.print("Port B Failed")
            print("[ERROR] Port B: {}".format(str(e)))
            raise
        
        # Test Port C (Lifting mechanism)
        try:
            self.lift_motor = Motor(Port.C)
            print("[OK] Port C lift motor found")
        except Exception as e:
            self.lift_motor = None
            print("[DEBUG] Port C lift motor not found, skipping lift commands")

        # Test port D (gate) 
        try:
            self.gate_motor = Motor(Port.D)
            print("[OK] Port D gate motor found")
        except Exception as e:
            self.gate_motor = None
            print("[DEBUG] Port D gate motor not found, skipping gate commands")    
        
        # Try DriveBase with minimal parameters
        try:
            # not working idk why
            self.robot = DriveBase(
                self.left_motor,
                self.right_motor,
                wheel_diameter=6.0,    # calibrated: 1000mm command → ~355mm actual at 17, so 17×(355/1000)=6.035≈6
                axle_track=118        # center-to-center between tracks: 100mm gap + ~18mm track width
            )
            print("[OK] DriveBase initialized")
            self.ev3.screen.clear()
            self.ev3.screen.print("Ready")
        except Exception as e:
            print("[ERROR] DriveBase failed: {}".format(str(e)))
            print("Trying without gyro...")
            # Fallback: use motors directly without DriveBase
            self.robot = None
            self.ev3.screen.clear()
            self.ev3.screen.print("Motor Mode")
        
        # Try to initialize gyro sensor for accurate turning (Port 1)
        try:
            self.gyro = GyroSensor(Port.S1)
            # Read once to confirm it's really a responsive gyro, not just a port
            start_angle = self.gyro.angle()
            print("[OK] Gyro sensor on Port.S1 (angle={})".format(start_angle))
        except Exception as e:
            self.gyro = None
            print("[DEBUG] Gyro sensor not found or failed ({}); using default turning".format(str(e)[:30]))
        
        self.forward_speed = 200
        self.turn_speed = 200
        self.lift_speed = 150
        
        self.commands_executed = 0
        self.commands_failed = 0
        self.mission_start_time = None
        self.last_command_time = None
        
        self._log("Motor initialization complete")
        self.ev3.speaker.say("Ready")
        self.ev3.light.on(Color.GREEN)
    
    def _log(self, message):
        """Log message with timestamp to EV3 screen and console"""
        current_time = time.time()
        
        # Format for screen (limited space)
        self.ev3.screen.clear()
        self.ev3.screen.print(message[:20])
        
        # Console output for debugging
        print("[EV3] {}".format(message))
    
    def execute_command(self, command_str):
        """Execute a single command and log execution timing"""
        command_str = command_str.strip().upper()
        
        # Skip empty lines and comments
        if not command_str or command_str.startswith("#"):
            return True
        
        cmd_start_time = time.time()
        command_display = command_str[:20]  # Truncate for display
        
        self.ev3.screen.clear()
        self.ev3.screen.print("Cmd: {}".format(command_display))
        
        try:
            # Parse command format: "TYPE:parameter"
            if ":" in command_str:
                cmd, value_str = command_str.split(":", 1)
                cmd = cmd.strip()
                value_str = value_str.strip()
                # For SAY commands, keep value as string; for others, convert to int
                if cmd == "SAY":
                    value = value_str
                else:
                    try:
                        value = int(value_str)
                    except ValueError:
                        self._log("Bad value: {}".format(value_str))
                        self.commands_failed += 1
                        return False
            else:
                cmd = command_str.strip()
                value = 0
            
            # Execute command based on type
            if cmd == "FORWARD":
                if value > 0:
                    self._log("FWD {} mm".format(value))
                    if self.robot:
                        self.robot.straight(-value / 3.2288)
                    else:
                        # Fallback: drive both motors in parallel
                        rotations = (value * 360) // 62  # Calibrated: 62mm per wheel rotation at 17mm diameter
                        self.left_motor.run_angle(-self.forward_speed, rotations, wait=False)
                        self.right_motor.run_angle(-self.forward_speed, rotations, wait=True)
                    self.commands_executed += 1
                    cmd_time = time.time() - cmd_start_time
                    self._log("OK ({:.1f}s)".format(cmd_time))
                else:
                    self._log("Bad distance: {}".format(value))
                    self.commands_failed += 1
                    return False
            
            elif cmd == "REVERSE":
                if value > 0:
                    self._log("REV {} mm".format(value))
                    if self.robot:
                        self.robot.straight(value)
                    else:
                        # Fallback: reverse both motors in parallel
                        rotations = (value * 360) // 174
                        self.left_motor.run_angle(self.forward_speed, rotations, wait=False)
                        self.right_motor.run_angle(self.forward_speed, rotations, wait=True)
                    self.commands_executed += 1
                    cmd_time = time.time() - cmd_start_time
                    self._log("OK ({:.1f}s)".format(cmd_time))
                else:
                    self._log("Bad distance: {}".format(value))
                    self.commands_failed += 1
                    return False
            
            elif cmd == "TURN":
                if value != 0:
                    self._log("TURN {} deg".format(value))
                    if self.gyro and self.robot:
                        # Closed-loop: turn in the commanded direction until the gyro
                        # has TRAVELED abs(value) degrees. Compared on magnitude so it
                        # works regardless of the gyro's sign convention.
                        print("[TURN] gyro closed-loop, target={} deg".format(value))
                        self.gyro.reset_angle(0)
                        target = abs(value)
                        rate = self.turn_speed if value > 0 else -self.turn_speed
                        last_logged = 0
                        while abs(self.gyro.angle()) < target:
                            self.robot.drive(0, rate)
                            traveled = abs(self.gyro.angle())
                            if traveled - last_logged >= 30:
                                print("[TURN] angle={}".format(self.gyro.angle()))
                                last_logged = traveled
                        self.robot.stop()
                        print("[TURN] done, final angle={}".format(self.gyro.angle()))
                    elif self.robot:
                        self.robot.turn(value)
                    else:
                        # Fallback: tank turn (both motors opposite directions in parallel)
                        # For continuous track, rotate both wheels in opposite directions
                        # Calibrated: 47 DriveBase degrees = 90 physical degrees
                        motor_angle = int(round(abs(value) * 736.5 / 90.0))
                        if value > 0:
                            # Turn right: left forward, right backward
                            self.left_motor.run_angle(self.turn_speed, motor_angle, wait=False)
                            self.right_motor.run_angle(-self.turn_speed, motor_angle, wait=True)
                        else:
                            # Turn left: left backward, right forward
                            self.left_motor.run_angle(-self.turn_speed, motor_angle, wait=False)
                            self.right_motor.run_angle(self.turn_speed, motor_angle, wait=True)
                    self.commands_executed += 1
                    cmd_time = time.time() - cmd_start_time
                    self._log("OK ({:.1f}s)".format(cmd_time))
                else:
                    self._log("Bad angle: {}".format(value))
                    self.commands_failed += 1
                    return False
            
            elif cmd == "SPEED":
                if value > 0:
                    self.forward_speed = value
                    self._log("Speed -> {} mm/s".format(value))
                    self.ev3.speaker.beep(frequency=1000, duration=100)
                    self.commands_executed += 1
                    return True
                else:
                    self._log("Bad speed: {}".format(value))
                    self.commands_failed += 1
                    return False
            
            elif cmd == "STOP":
                self._log("STOP")
                if self.robot:
                    self.robot.stop()
                else:
                    self.left_motor.stop()
                    self.right_motor.stop()
                self.ev3.speaker.beep(frequency=500, duration=200)
                self.commands_executed += 1
                return True
            
            elif cmd == "SAY":
                if value:
                    self._log("SAY: {}".format(value[:15]))
                    # Queue voice command to run in background
                    def speak_async():
                        self.voice_speaking = True
                        try:
                            self.ev3.speaker.say(value)
                        except:
                            pass
                        self.voice_speaking = False
                    thread = threading.Thread(target=speak_async)
                    thread.daemon = True
                    thread.start()
                    self.commands_executed += 1
                    return True
                else:
                    self._log("No text to say")
                    self.commands_failed += 1
                    return False
            
            elif cmd == "SPIN_START":
                self._log("SPINNING!")
                self.spinning = True
                def spin_forever():
                    while self.spinning:
                        try:
                            if self.robot:
                                self.robot.turn(360)
                            else:
                                # Tank spin: both motors opposite directions
                                self.left_motor.run_angle(self.spin_speed, 360, wait=False)
                                self.right_motor.run_angle(-self.spin_speed, 360, wait=True)
                        except:
                            pass
                        time.sleep(0.1)
                thread = threading.Thread(target=spin_forever)
                thread.daemon = True
                thread.start()
                self.commands_executed += 1
                return True
            
            elif cmd == "SPIN_STOP":
                self._log("SPIN STOP")
                self.spinning = False
                if self.robot:
                    self.robot.stop()
                else:
                    self.left_motor.stop()
                    self.right_motor.stop()
                self.commands_executed += 1
                return True
            
            elif cmd == "LIFT_UP":
                if self.lift_motor:
                    if value > 0:
                        self._log("LIFT UP {} deg".format(value))
                        # Calibrated: 130 motor degrees = 45 physical degrees
                        scaled = value #int(round(value * 130.0 / 45.0))
                        self.lift_motor.run_angle(-self.lift_speed, scaled)
                        self.commands_executed += 1
                        cmd_time = time.time() - cmd_start_time
                        self._log("OK ({:.1f}s)".format(cmd_time))
                    else:
                        self._log("Bad angle: {}".format(value))
                        self.commands_failed += 1
                        return False
                else:
                    self._log("Lift motor N/A")
                    self.commands_failed += 1
                    return False
            
            elif cmd == "LIFT_DOWN":
                if self.lift_motor:
                    if value > 0:
                        self._log("LIFT DOWN {} deg".format(value))
                        # Calibrated: 130 motor degrees = 45 physical degrees
                        scaled = int(round(value * 130.0 / 45.0))
                        self.lift_motor.run_angle(self.lift_speed, scaled)
                        self.commands_executed += 1
                        cmd_time = time.time() - cmd_start_time
                        self._log("OK ({:.1f}s)".format(cmd_time))
                    else:
                        self._log("Bad angle: {}".format(value))
                        self.commands_failed += 1
                        return False
                else:
                    self._log("Lift motor N/A")
                    self.commands_failed += 1
                    return False
                
            elif cmd == "GATE_OPEN":
                if self.gate_motor:
                    self._log("GATE OPEN {} deg".format(value))
                    self.gate_motor.run_angle(-200, value)
                    self.commands_executed += 1
                    return True
                else:
                    self._log("Gate motor N/A")
                    self.commands_failed += 1
                    return False

            elif cmd == "GATE_CLOSE":
                if self.gate_motor:
                    self._log("GATE CLOSE {} deg".format(value))
                    self.gate_motor.run_angle(200, value)
                    self.commands_executed += 1
                    return True
                else:
                    self._log("Gate motor N/A")
                    self.commands_failed += 1
                    return False
            
            elif cmd == "SHAKE_LIFT":
                if self.lift_motor:
                    self._log("SHAKE LIFT")
                    for _ in range(3):  # Shake 3 times, adjust as needed
                        self.lift_motor.run_angle(-self.lift_speed, 90)
                        self.lift_motor.run_angle(self.lift_speed, 90)
                    self.commands_executed += 1
                    return True
                else:
                    self._log("Lift motor N/A")
                    self.commands_failed += 1
                    return False


            

            else:
                self._log("Unknown: {}".format(cmd))
                self.commands_failed += 1
                return False
            
            return True
        
        except Exception as e:
            self._log("Error: {}".format(str(e)[:15]))
            self.commands_failed += 1
            return False
        
    
    
    def execute_mission(self, mission_file="commands.txt"):
        """Read mission file and execute all commands in sequence"""
        self.mission_start_time = time.time()
        self.ev3.screen.clear()
        self.ev3.speaker.say("Starting mission")
        self.ev3.light.on(Color.YELLOW)
        
        # Try to find mission file in multiple locations
        possible_paths = [
            mission_file,
            "/home/root/{}".format(mission_file),
            "/home/robot/{}".format(mission_file),
            "/home/robot/CDIO-Mindstorms/{}".format(mission_file),
        ]
        
        commands = []
        found_file = None
        
        for path in possible_paths:
            try:
                with open(path, "r") as f:
                    file_lines = f.readlines()
                    for line in file_lines:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            commands.append(line)
                    if commands:
                        found_file = path
                        break
            except OSError:
                continue
        
        # Handle file not found
        if not found_file:
            self.ev3.screen.clear()
            self.ev3.screen.print("No mission file!")
            self.ev3.screen.print("Searched: commands.txt")
            self._log("Mission file not found")
            self.ev3.speaker.say("Error")
            self.ev3.light.on(Color.RED)
            return False
        
        # Handle empty mission
        if not commands:
            self.ev3.screen.clear()
            self.ev3.screen.print("Empty mission!")
            self._log("No commands in file")
            self.ev3.speaker.say("Error")
            self.ev3.light.on(Color.RED)
            return False
        
        self.ev3.light.on(Color.GREEN)
        self._log("Loaded {} cmds".format(len(commands)))
        self.ev3.speaker.say("Mission loaded")
        time.sleep(2)
        
        # Execute all commands
        try:
            for i, cmd in enumerate(commands):
                progress = "Step {}/{}".format(i + 1, len(commands))
                self.ev3.screen.clear()
                self.ev3.screen.print(progress)
                self.ev3.screen.print(cmd[:20])
                
                success = self.execute_command(cmd)
                
                if not success and cmd != "STOP":
                    self._log("Cmd failed!")
                    time.sleep(1)
            
            # Mission complete
            mission_time = time.time() - self.mission_start_time
            self.ev3.screen.clear()
            self.ev3.screen.print("Mission OK!")
            self.ev3.screen.print("Time: {:.0f}s".format(mission_time))
            self._log("Mission complete!")
            self.ev3.speaker.say("Done")
            self.ev3.light.on(Color.GREEN)
            
            print("[SUMMARY] Executed: {} | Failed: {} | Time: {:.1f}s".format(
                self.commands_executed, self.commands_failed, mission_time))
            return True

        except Exception as e:
            self.ev3.screen.clear()
            self.ev3.screen.print("Mission Error!")
            self._log("Exception: {}".format(str(e)[:15]))
            self.ev3.speaker.say("Error")
            self.ev3.light.on(Color.RED)
            print("[ERROR] Mission failed: {}".format(str(e)))
            return False
        
        finally:
            if self.robot:
                self.robot.stop()
            else:
                self.left_motor.stop()
                self.right_motor.stop()
            if self.lift_motor:
                self.lift_motor.stop()


def run_diagnostics():
    """Test which motors are connected"""
    print("=" * 60)
    print("GolfBot 9000: Motor Diagnostics")
    print("=" * 60)
    
    ev3 = EV3Brick()
    ev3.speaker.say("Diagnostics")
    
    ports = [("Port.A", Port.A), ("Port.B", Port.B), ("Port.C", Port.C), ("Port.D", Port.D)]
    
    print("\nTesting motors on each port...\n")
    
    for port_name, port in ports:
        try:
            motor = Motor(port)
            print("[FOUND] Motor on {}".format(port_name))
            ev3.speaker.beep(frequency=1000, duration=100)
            # Try to move it slightly to confirm
            motor.run_angle(100, 90, wait=True)
            motor.reset_angle(0)
            print("  → Motor responsive (moved 90°)")
        except Exception as e:
            print("[NOT FOUND] {} - {}".format(port_name, str(e)[:40]))
    
    print("\nSensor ports:\n")
    sensor_ports = [("Port.S1", Port.S1), ("Port.S2", Port.S2), ("Port.S3", Port.S3), ("Port.S4", Port.S4)]
    
    for port_name, port in sensor_ports:
        try:
            # Try gyro first (most likely)
            gyro = GyroSensor(port)
            print("[FOUND] Gyro Sensor on {}".format(port_name))
            ev3.speaker.beep(frequency=800, duration=100)
        except:
            try:
                # Try color sensor
                color = ColorSensor(port)
                print("[FOUND] Color Sensor on {}".format(port_name))
                ev3.speaker.beep(frequency=800, duration=100)
            except:
                print("[EMPTY] {}".format(port_name))
    
    print("\nDiagnostics complete.")
    print("Expected configuration:")
    print("  - Large Motor on Port.A (left)")
    print("  - Large Motor on Port.B (right)")
    print("  - Optional: Gyro on Port.S4")


def main():
    """Main entry point"""
    import sys
    
    print("\n" + "=" * 60)
    print("GolfBot 9000: EV3 Navigation Controller")
    print("=" * 60 + "\n")
    
    # Check for diagnostic mode
    if "--diagnostics" in sys.argv or "--test" in sys.argv:
        try:
            run_diagnostics()
        except Exception as e:
            print("[ERROR] Diagnostics failed: {}".format(str(e)))
        return
    
    # Normal mission execution
    try:
        controller = EV3NavController()
        controller.execute_mission()
    except Exception as e:
        print("[FATAL ERROR] {}".format(str(e)))
        print("\nIf motors not found:")
        print("  1. Check motor connections")
        print("  2. Run with: python3 main.py --diagnostics")
        print("  3. Verify motors are on Ports A and B")


if __name__ == "__main__":
    main()
