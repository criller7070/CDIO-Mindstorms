#!/usr/bin/env pybricks-micropython

"""
Entry point for EV3 brick. Reads commands from commands.txt and executes a full mission
Commands: 
    FORWARD:distance
    TURN:angle
    REVERSE:distance
    SPEED:value
    STOP
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.robotics import DriveBase
import time
import threading
import os

# constants
WHEEL_DIAMETER          = 6.0    # mm  - effective rolling diameter of tracks
AXLE_TRACK              = 43     # mm  - effective turn radius (empirical; physical is 118 mm but tracks slip)
FORWARD_SPEED           = 200    # mm/s  - default forward speed
TURN_SPEED              = 200    # deg/s - default turn rate
LIFT_SPEED              = 200    # deg/s - lift motor speed
SPIN_SPEED              = 300    # deg/s - spin motor speed
GATE_SPEED              = 200    # deg/s - gate motor speed
GYRO_BRAKE_OFFSET       = 12     # deg  - stop gyro loop this many degrees early to account for motor inertia
FORWARD_MM_PER_ROTATION = 62     # mm per motor rotation - actual FORWARD implementation (DriveBase is wired but not used for movement)
REVERSE_MM_PER_ROTATION = 174    # mm per motor rotation - different from forward because the tracks grip differently going back
FALLBACK_TURN_RATIO     = 736.5 / 90.0  # motor degrees per physical degree (fallback tank turn)
LIFT_DOWN_MOTOR_RATIO   = 130.0 / 45.0  # motor degrees per physical degree (LIFT_DOWN)


class EV3NavController:
    def __init__(self):
        """1. Initialize EV3 robot"""
        self.ev3 = EV3Brick()
        self.ev3.screen.clear()
        self.ev3.screen.print("Init...")
        
        # 2. Spinning control
        self.spinning = False
        self.spin_speed = SPIN_SPEED

        # 4a. Test Port A
        try:
            self.left_motor = Motor(Port.A)
            print("[OK] Port A motor found")
        except Exception as e:
            self.ev3.screen.clear()
            self.ev3.screen.print("Port A Failed")
            print("[ERROR] Port A: {}".format(str(e)))
            raise
        
        # 4b. Test Port B
        try:
            self.right_motor = Motor(Port.B)
            print("[OK] Port B motor found")
        except Exception as e:
            self.ev3.screen.clear()
            self.ev3.screen.print("Port B Failed")
            print("[ERROR] Port B: {}".format(str(e)))
            raise
        
        # 4c. Test Port C (Lifting mechanism)
        try:
            self.lift_motor = Motor(Port.C)
            print("[OK] Port C lift motor found")
        except Exception as e:
            self.lift_motor = None
            print("[DEBUG] Port C lift motor not found, skipping lift commands")

        # 4d. Test Port D (gate)
        try:
            self.gate_motor = Motor(Port.D)
            print("[OK] Port D gate motor found")
        except Exception as e:
            self.gate_motor = None
            print("[DEBUG] Port D gate motor not found, skipping gate commands")

        if self.gate_motor:
            # gate must be manually placed at closed position before starting.
            # zero encoder here so 90=closed, 0=open throughout the run.
            self.gate_motor.reset_angle(90)
            print("[OK] Gate encoder zeroed at closed position (angle=90)")


        # 5. Initialize DriveBase
        try:
            self.robot = DriveBase(
                self.left_motor,
                self.right_motor,
                wheel_diameter=WHEEL_DIAMETER,
                axle_track=AXLE_TRACK
            )
            print("[OK] DriveBase initialized")
            self.ev3.screen.clear()
            self.ev3.screen.print("Ready")
        except Exception as e:
            print("[DEBUG] DriveBase unavailable ({}), using motor.run_angle".format(str(e)))
            self.robot = None

        # 6. Motor self-test (TODO: move to run_diagnostics() - runs on every startup incl. follow mode)
        for name, motor in [("A", self.left_motor), ("B", self.right_motor)]:
            results = []
            for deg, label in [(180, "fwd"), (-180, "rev")]:
                before = motor.angle()
                try:
                    motor.run_angle(200, abs(deg), wait=True)
                except Exception as e:
                    results.append("{}:ERR({})".format(label, str(e)[:15]))
                    continue
                moved = abs(motor.angle() - before)
                if moved < 90:
                    results.append("{}:STALL({}deg)".format(label, moved))
                else:
                    results.append("{}:OK({}deg)".format(label, moved))
            motor.stop()
            status = ", ".join(results)
            if "STALL" in status or "ERR" in status:
                print("[WARN] Motor {} self-test FAILED: {}".format(name, status))
            else:
                print("[OK] Motor {} self-test: {}".format(name, status))

        # 7. Gyro sensor (S1) - optional but gives much better turns
        try:
            self.gyro = GyroSensor(Port.S1)
            self.gyro.reset_angle(0)
            print("[OK] Gyro sensor on Port.S1")
        except Exception as e:
            self.gyro = None
            print("[DEBUG] Gyro sensor not found or failed ({}); using default turning".format(str(e)[:30]))
        
        # 8. Default speeds
        self.forward_speed = FORWARD_SPEED
        self.turn_speed = TURN_SPEED
        self.lift_speed = LIFT_SPEED

        # 9. Stats + battery check
        self.commands_executed = 0
        self.commands_failed = 0
        self.mission_start_time = None
        self.last_command_time = None

        batt_mv = self.ev3.battery.voltage()
        print("[BATT] {:.2f}V".format(batt_mv / 1000.0))

        self._log("Motor initialization complete")
        self.ev3.speaker.say("Ready")
        self.ev3.light.on(Color.GREEN)
    
    def _log(self, message):
        """print to screen (truncated) and console"""
        current_time = time.time()

        self.ev3.screen.clear()
        self.ev3.screen.print(message[:20])

        print("[EV3] {}".format(message))
    
    def execute_command(self, command_str):
        """run one command, return True on success"""
        command_str = command_str.strip().upper()

        # skip blanks and comments
        if not command_str or command_str.startswith("#"):
            return True

        cmd_start_time = time.time()
        command_display = command_str[:20]  # truncate for display
        
        self.ev3.screen.clear()
        self.ev3.screen.print("Cmd: {}".format(command_display))
        
        try:
            # split "TYPE:value"
            if ":" in command_str:
                cmd, value_str = command_str.split(":", 1)
                cmd = cmd.strip()
                value_str = value_str.strip()
                # SAY keeps its value as a string; everything else is an int
                if cmd in ("SAY", "DROPOFF"):
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
            
            # dispatch
            if cmd == "FORWARD":
                if value > 0:
                    self._log("FWD {} mm".format(value))
                    rotations = (value * 360) // FORWARD_MM_PER_ROTATION
                    enc_l0 = self.left_motor.angle()
                    enc_r0 = self.right_motor.angle()
                    try:
                        self.left_motor.run_angle(-self.forward_speed, rotations, wait=False)
                        self.right_motor.run_angle(-self.forward_speed, rotations, wait=True)
                    except OSError as e:
                        print("[FWD] OSError {}, resetting motors".format(e))
                        self.left_motor.stop()
                        self.right_motor.stop()
                        time.sleep(0.2)
                        self.left_motor.run_angle(-self.forward_speed, rotations, wait=False)
                        self.right_motor.run_angle(-self.forward_speed, rotations, wait=True)
                    enc_l1 = self.left_motor.angle()
                    enc_r1 = self.right_motor.angle()
                    print("[FWD] enc L:{}->{} ({}) R:{}->{} ({}) cmd_rot={}".format(
                        enc_l0, enc_l1, enc_l1 - enc_l0,
                        enc_r0, enc_r1, enc_r1 - enc_r0, rotations))
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
                    rotations = (value * 360) // REVERSE_MM_PER_ROTATION
                    enc_l0 = self.left_motor.angle()
                    enc_r0 = self.right_motor.angle()
                    self.left_motor.run_angle(self.forward_speed, rotations, wait=False)
                    self.right_motor.run_angle(self.forward_speed, rotations, wait=True)
                    enc_l1 = self.left_motor.angle()
                    enc_r1 = self.right_motor.angle()
                    print("[REV] enc L:{}->{} ({}) R:{}->{} ({}) cmd_rot={}".format(
                        enc_l0, enc_l1, enc_l1 - enc_l0,
                        enc_r0, enc_r1, enc_r1 - enc_r0, rotations))
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
                        gyro_before = self.gyro.angle()
                        self.gyro.reset_angle(0)
                        target = abs(value)
                        print("[TURN] gyro CL target={} gyro_before={}".format(value, gyro_before))
                        last_logged = 0
                        t0 = time.time()
                        timeout = max(target / 10.0, 1.0) * 3.0 + 3.0
                        self.left_motor.stop()
                        self.right_motor.stop()
                        enc_l0 = self.left_motor.angle()
                        enc_r0 = self.right_motor.angle()
                        try:
                            if value > 0:
                                self.left_motor.run(self.turn_speed)
                                self.right_motor.run(-self.turn_speed)
                            else:
                                self.left_motor.run(-self.turn_speed)
                                self.right_motor.run(self.turn_speed)
                        except OSError as e:
                            print("[TURN] OSError {}, resetting motors".format(e))
                            self.left_motor.stop()
                            self.right_motor.stop()
                            time.sleep(0.2)
                            if value > 0:
                                self.left_motor.run(self.turn_speed)
                                self.right_motor.run(-self.turn_speed)
                            else:
                                self.left_motor.run(-self.turn_speed)
                                self.right_motor.run(self.turn_speed)
                        while abs(self.gyro.angle()) < target - GYRO_BRAKE_OFFSET:
                            if time.time() - t0 > timeout:
                                print("[TURN] TIMEOUT gyro={} target={}".format(
                                    self.gyro.angle(), target))
                                break
                            traveled = abs(self.gyro.angle())
                            if traveled - last_logged >= 5:
                                print("[TURN] gyro={}".format(self.gyro.angle()))
                                last_logged = traveled
                        self.left_motor.stop()
                        self.right_motor.stop()
                        enc_l1 = self.left_motor.angle()
                        enc_r1 = self.right_motor.angle()
                        t_elapsed = time.time() - t0
                        overshoot = abs(self.gyro.angle()) - target
                        print("[TURN] done gyro={} target={} overshoot={} enc_L={} enc_R={} t={:.1f}s".format(
                            self.gyro.angle(), value, overshoot,
                            enc_l1 - enc_l0, enc_r1 - enc_r0, t_elapsed))
                    else:
                        motor_angle = int(round(abs(value) * FALLBACK_TURN_RATIO))
                        if value > 0:
                            self.left_motor.run_angle(self.turn_speed, motor_angle, wait=False)
                            self.right_motor.run_angle(-self.turn_speed, motor_angle, wait=True)
                        else:
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
                    def speak_async():
                        try:
                            self.ev3.speaker.say(value)
                        except:
                            pass
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
                                # tank spin
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
                        # 130 motor deg = 45 physical deg (measured)
                        scaled = value  # motor deg == physical deg on UP side
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
                        scaled = value
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
                    self._log("GATE -> {}".format(value))
                    try:
                        self.gate_motor.run_target(GATE_SPEED, value)
                    except Exception as e:
                        self._log("Gate open error: {}".format(e))
                        self.commands_failed += 1
                        return False
                    self.commands_executed += 1
                    return True
                else:
                    self._log("Gate motor N/A")
                    self.commands_failed += 1
                    return False

            elif cmd == "GATE_CLOSE":
                if self.gate_motor:
                    self._log("GATE -> {}".format(value))
                    try:
                        self.gate_motor.run_target(GATE_SPEED, value)
                    except Exception as e:
                        self._log("Gate close error: {}".format(e))
                        self.commands_failed += 1
                        return False
                    self.commands_executed += 1
                    return True
                else:
                    self._log("Gate motor N/A")
                    self.commands_failed += 1
                    return False
            
            elif cmd == "DROPOFF":
                # gate_deg:lift_deg - open gate and lift tray simultaneously
                parts = value.split(":")
                gate_deg = int(parts[0]) if len(parts) > 0 and parts[0] else 90
                lift_deg = int(parts[1]) if len(parts) > 1 and parts[1] else 150
                self._log("DROPOFF gate={} lift={}".format(gate_deg, lift_deg))
                if self.gate_motor:
                    self.gate_motor.run_target(GATE_SPEED, gate_deg, wait=False)
                if self.lift_motor:
                    self.lift_motor.run_angle(-self.lift_speed, lift_deg)
                self.commands_executed += 1

            elif cmd == "SHAKE_LIFT":
                if self.lift_motor:
                    self._log("SHAKE LIFT")
                    for _ in range(3):  # 3 shakes, tune as needed
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
        
    
    
    # IPC files shared with ev3_server.py
    CL_CMD_FILE = "/home/robot/cl_cmd.txt"
    CL_ACK_FILE = "/home/robot/cl_ack.txt"

    def run_follow_loop(self):
        """wait for commands from ev3_server.py and execute them one at a time."""
        self.ev3.screen.clear()
        self.ev3.screen.print("Follow mode")
        self.ev3.speaker.say("Follow mode")
        self.ev3.light.on(Color.GREEN)

        self.turn_speed = 300
        self._log("Follow loop ready")

        # stale ack would look like a real one without this
        self._write_ack(-1)
        last_seq = -1

        while True:
            seq, cmd = self._read_command()
            if seq is None or seq == last_seq or seq < 0:
                time.sleep(0.02)
                continue

            self.ev3.screen.clear()
            self.ev3.screen.print("#{}: {}".format(seq, cmd[:14]))
            self.execute_command(cmd)
            last_seq = seq
            self._write_ack(seq)

    def _read_command(self):
        """return (seq, command) from CL_CMD_FILE, or (None, None) on any error."""
        try:
            with open(self.CL_CMD_FILE, "r") as f:
                line = f.readline().strip()
            parts = line.split(" ", 1)
            seq = int(parts[0])
            cmd = parts[1] if len(parts) > 1 else ""
            return seq, cmd
        except (OSError, ValueError, IndexError):
            return None, None

    def _write_ack(self, seq):
        """write seq to CL_ACK_FILE atomically."""
        tmp = self.CL_ACK_FILE + ".tmp"
        try:
            with open(tmp, "w") as f:
                f.write("{} DONE\n".format(seq))
            os.rename(tmp, self.CL_ACK_FILE)
        except OSError as e:
            self._log("ack write failed: {}".format(str(e)[:15]))

    def execute_mission(self, mission_file="commands.txt"):
        """load mission file and run all commands in order"""
        self.mission_start_time = time.time()
        self.ev3.screen.clear()
        self.ev3.speaker.say("Starting mission")
        self.ev3.light.on(Color.YELLOW)
        
        # TODO: simplify to one path - robot always runs from /home/robot/CDIO-Mindstorms/robot/
        # this multi-path search papers over a bad deployment rather than catching it
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
        
        # no file found
        if not found_file:
            self.ev3.screen.clear()
            self.ev3.screen.print("No mission file!")
            self.ev3.screen.print("Searched: commands.txt")
            self._log("Mission file not found")
            self.ev3.speaker.say("Error")
            self.ev3.light.on(Color.RED)
            return False
        
        # file found but nothing in it
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
        
        # run
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
            
            # done
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
    """probe each port and report what's connected"""
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
            # nudge it to confirm it's not just detected but actually responsive
            motor.run_angle(100, 90, wait=True)
            motor.reset_angle(0)
            print("  to Motor responsive (moved 90 deg)")
        except Exception as e:
            print("[NOT FOUND] {} - {}".format(port_name, str(e)[:40]))
    
    print("\nSensor ports:\n")
    sensor_ports = [("Port.S1", Port.S1), ("Port.S2", Port.S2), ("Port.S3", Port.S3), ("Port.S4", Port.S4)]
    
    for port_name, port in sensor_ports:
        try:
            # gyro first (most common)
            gyro = GyroSensor(port)
            print("[FOUND] Gyro Sensor on {}".format(port_name))
            ev3.speaker.beep(frequency=800, duration=100)
        except:
            try:
                # try color sensor
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
    import sys

    print("\n" + "=" * 60)
    print("GolfBot 9000: EV3 Navigation Controller")
    print("=" * 60 + "\n")

    # diagnostic / test mode
    if "--diagnostics" in sys.argv or "--test" in sys.argv:
        try:
            run_diagnostics()
        except Exception as e:
            print("[ERROR] Diagnostics failed: {}".format(str(e)))
        return

    # follow mode: one command at a time, streamed from the PC
    if "--follow" in sys.argv:
        try:
            controller = EV3NavController()
            controller.run_follow_loop()
        except Exception as e:
            print("[FATAL ERROR] {}".format(str(e)))
        return

    # open-loop mission from commands.txt
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
