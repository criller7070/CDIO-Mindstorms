#!/usr/bin/env pybricks-micropython

"""
EV3 Navigation Controller - Autonomous Path Execution
Reads commands from commands.txt and executes a full mission
Commands: FORWARD:distance, TURN:angle, REVERSE:distance, SPEED:value, STOP
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
import time

class EV3NavController:
    def __init__(self):
        """Initialize EV3 robot"""
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        
        self.robot = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=55.5,
            axle_track=104
        )
        
        self.forward_speed = 200
        self.turn_speed = 90
        
        self.ev3.speaker.say("Ready")
        self.ev3.light.on(Color.GREEN)
    
    def execute_command(self, command_str):
        """Execute a single command"""
        command_str = command_str.strip().upper()
        
        if not command_str or command_str.startswith("#"):
            return
        
        self.ev3.screen.clear()
        self.ev3.screen.print("{}".format(command_str[:12]))
        
        if ":" in command_str:
            cmd, value = command_str.split(":", 1)
            cmd = cmd.strip()
            try:
                val = int(value.strip())
            except:
                val = 0
        else:
            cmd = command_str
            val = 0
        
        if cmd == "FORWARD":
            if val > 0:
                self.robot.straight(val)
            else:
                self.robot.drive(self.forward_speed, 0)
        
        elif cmd == "REVERSE":
            if val > 0:
                self.robot.straight(-val)
            else:
                self.robot.drive(-self.forward_speed, 0)
        
        elif cmd == "TURN":
            if val != 0:
                self.robot.turn(val)
        
        elif cmd == "TURN_LEFT":
            self.robot.turn(-45)
        
        elif cmd == "TURN_RIGHT":
            self.robot.turn(45)
        
        elif cmd == "STOP":
            self.robot.stop()
            self.ev3.speaker.beep(frequency=500, duration=200)
        
        elif cmd == "SPEED":
            self.forward_speed = val if val > 0 else 200
            self.ev3.speaker.beep(frequency=1000, duration=100)
        
        else:
            self.ev3.screen.print("Unknown: {0}".format(cmd))
    
    def execute_mission(self):
        """Read mission file and execute all commands"""
        self.ev3.screen.clear()
        self.ev3.speaker.say("Starting mission")
        self.ev3.light.on(Color.YELLOW)
        
        try:
            commands = []
            try:
                with open("/home/robot/CDIO-Mindstorms/commands.txt", "r") as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            commands.append(line)
            except OSError:
                self.ev3.screen.clear()
                self.ev3.screen.print("No mission file!")
                self.ev3.speaker.say("Error")
                self.ev3.light.on(Color.RED)
                return
            
            if not commands:
                self.ev3.screen.clear()
                self.ev3.screen.print("Empty mission!")
                return
            
            self.ev3.light.on(Color.GREEN)
            self.ev3.speaker.say("Mission loaded")
            time.sleep(2)
            
            for i, cmd in enumerate(commands):
                self.ev3.screen.clear()
                self.ev3.screen.print("Step {0}/{1}".format(i+1, len(commands)))
                self.ev3.screen.print(cmd[:15])
                self.execute_command(cmd)
            
            self.ev3.screen.clear()
            self.ev3.screen.print("Mission complete!")
            self.ev3.speaker.say("Done")
            self.ev3.light.on(Color.GREEN)

        except Exception as e:
            self.ev3.screen.clear()
            self.ev3.screen.print("Error!")
            self.ev3.speaker.say("Error")
            self.ev3.light.on(Color.RED)
        
        finally:
            self.robot.stop()


controller = EV3NavController()
controller.execute_mission()
