#!/usr/bin/env python3
"""
Simple EV3 receiver program that listens for Bluetooth commands and plays sounds.
Run this on your EV3 device.
"""

import bluetooth
import time
import subprocess

class EV3SoundReceiver:
    def start_listening(self):
        """Start listening for Bluetooth commands"""
        # Create Bluetooth socket
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.bind(("", bluetooth.PORT_ANY))
        sock.listen(1)
        
        print("EV3 Sound Receiver Ready")
        print("Waiting for Bluetooth connection...")
        
        try:
            client_sock, client_info = sock.accept()
            print(f"Connected: {client_info}")
            
            while True:
                data = client_sock.recv(1024)
                if not data:
                    break
                
                command = data.decode('utf-8').strip()
                print(f"Received: {command}")
                self.execute_command(command)
        
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            client_sock.close()
            sock.close()
    
    def execute_command(self, command):
        """Execute the received command"""
        if command == "SOUND":
            self.play_sound()
        elif command == "BEEP":
            self.beep()
    
    def play_sound(self):
        """Play a sound using ev3dev or system command"""
        try:
            # Try using ev3dev2
            from ev3dev2.sound import Sound
            sound = Sound()
            sound.beep()
            print("Sound played")
        except:
            try:
                # Fallback: use Linux beep command
                subprocess.run(['beep'], timeout=1)
                print("Beep executed")
            except:
                print("Could not play sound")
    
    def beep(self):
        """Play a beep sound"""
        try:
            from ev3dev2.sound import Sound
            sound = Sound()
            sound.beep(frequency=1000, duration=500)
            print("Beep played")
        except:
            print("Could not play beep")

if __name__ == "__main__":
    receiver = EV3SoundReceiver()
    receiver.start_listening()
