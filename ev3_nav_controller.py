#!/usr/bin/env pybricks-micropython

"""
EV3 Bluetooth Command Listener for Vision-Based Navigation
Receives steering commands from host PC and executes robot movements.
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import time

class EV3NavController:
    def __init__(self):
        """Initialize EV3 robot and communication"""
        # Initialize EV3 Brick
        self.ev3 = EV3Brick()
        
        # Initialize motors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        
        # Initialize drive base
        self.robot = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=55.5,  # Adjust based on your wheel size
            axle_track=104         # Adjust based on your wheelbase
        )
        
        # Movement parameters
        self.forward_speed = 200    # mm/s
        self.turn_speed = 90        # deg/s
        self.turn_angle = 45        # degrees to turn
        
        self.ev3.speaker.say("Navigation ready")
        self.ev3.light.on(Color.GREEN)
    
    def execute_command(self, command):
        """Execute movement based on received command"""
        command = command.strip().upper()
        
        self.ev3.screen.print(f"CMD: {command}")
        
        if command == "FORWARD":
            self.move_forward()
        
        elif command == "STOP":
            self.stop()
        
        elif command == "TURN_LEFT":
            self.turn_left()
        
        elif command == "TURN_RIGHT":
            self.turn_right()
        
        elif command == "REVERSE":
            self.move_backward()
        
        elif command.startswith("SPEED"):
            # Format: SPEED:500 to set speed
            try:
                speed = int(command.split(":")[1])
                self.forward_speed = speed
                self.ev3.speaker.beep(frequency=1000, duration=100)
            except:
                self.ev3.speaker.say("Invalid speed")
        
        else:
            self.ev3.screen.print(f"Unknown: {command}")
    
    def move_forward(self):
        """Move forward at set speed"""
        self.robot.drive(self.forward_speed, 0)
    
    def move_backward(self):
        """Move backward"""
        self.robot.drive(-self.forward_speed, 0)
    
    def stop(self):
        """Stop all motors"""
        self.robot.stop()
        self.ev3.speaker.beep(frequency=500, duration=200)
    
    def turn_left(self):
        """Turn left"""
        self.robot.turn(-self.turn_angle)
        self.ev3.speaker.beep(frequency=800)
    
    def turn_right(self):
        """Turn right"""
        self.robot.turn(self.turn_angle)
        self.ev3.speaker.beep(frequency=800)
    
    def start_listening(self):
        """Start listening for Bluetooth commands"""
        import bluetooth
        from pybricks.parameters import Color
        
        self.ev3.screen.clear()
        self.ev3.speaker.say("Starting Bluetooth listener")
        
        try:
            # Create Bluetooth socket
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.bind(("", bluetooth.PORT_ANY))
            sock.listen(1)
            
            self.ev3.screen.print("Waiting for connection...")
            self.ev3.light.on(Color.YELLOW)
            
            # Accept incoming connection
            client_sock, client_info = sock.accept()
            
            self.ev3.screen.print("Connected!")
            self.ev3.light.on(Color.GREEN)
            self.ev3.speaker.say("Connected")
            
            command_count = 0
            
            while True:
                try:
                    data = client_sock.recv(1024)
                    
                    if not data:
                        break
                    
                    command = data.decode('utf-8').strip()
                    self.ev3.screen.print(f"#{command_count}: {command[:10]}")
                    
                    self.execute_command(command)
                    command_count += 1
                
                except Exception as e:
                    self.ev3.screen.print(f"Error: {str(e)[:20]}")
                    break
        
        except Exception as e:
            self.ev3.screen.print(f"BT Error: {str(e)[:20]}")
            self.ev3.speaker.say("Bluetooth error")
            self.ev3.light.on(Color.RED)
        
        finally:
            try:
                client_sock.close()
                sock.close()
            except:
                pass
            
            self.robot.stop()
            self.ev3.screen.print("Disconnected")
            self.ev3.speaker.say("Disconnected")


if __name__ == "__main__":
    from pybricks.parameters import Color
    
    controller = EV3NavController()
    controller.start_listening()
