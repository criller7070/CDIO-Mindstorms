#!/usr/bin/env python3
"""
Host PC Bluetooth Mission Sender
Loads commands from commands.txt and sends them to EV3 via Bluetooth
Monitors execution and logs all activity

⚠️  IMPORTANT: Run this on HOST PC (Windows/Mac/Linux), NOT on EV3!
"""

import socket
import subprocess
import re
import time
import sys
import platform
import os


def is_running_on_ev3():
    """Detect if running on EV3 or Host PC"""
    # EV3 specific indicators
    try:
        # Check for ev3dev or pybricks environment
        if 'ev3dev' in platform.platform().lower():
            return True
        if os.path.exists('/home/robot'):
            return True
        if os.path.exists('/usr/bin/ev3-language'):
            return True
    except:
        pass
    return False


class HostMissionSender:
    def __init__(self, ev3_address=None):
        """Initialize host-side mission sender"""
        self.ev3_address = ev3_address
        self.sock = None
        self.commands_sent = 0
        self.commands_failed = 0
        self.mission_start_time = None
        self.connection_active = False
    
    def find_ev3_devices(self):
        """Find paired Bluetooth EV3 devices on Windows"""
        try:
            result = subprocess.run(
                ["reg", "query", "HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Services\\BTHPORT\\Parameters\\Devices"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5,
                universal_newlines=True
            )
            addresses = re.findall(r'[0-9A-Fa-f]{12}', result.stdout)
            if addresses:
                formatted = []
                for addr in addresses:
                    fmt = ':'.join([addr[i:i+2] for i in range(0, 12, 2)])
                    formatted.append(fmt)
                return formatted
        except Exception as e:
            self._log("ERROR: Could not query Bluetooth devices: {}".format(str(e)))
        
        return []
    
    def _log(self, message):
        """Log message with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        log_msg = "[{}] {}".format(timestamp, message)
        print(log_msg)
    
    def connect_to_ev3(self, target_address=None):
        """Establish Bluetooth connection to EV3"""
        if target_address:
            devices = [target_address]
            self._log("Attempting to connect to specified EV3: {}".format(target_address))
        else:
            self._log("Searching for EV3 devices...")
            devices = self.find_ev3_devices()
            
            if not devices:
                self._log("ERROR: No Bluetooth devices found!")
                return False
            
            self._log("Found {} Bluetooth device(s):".format(len(devices)))
            for i, dev in enumerate(devices, 1):
                self._log("  [{}] {}".format(i, dev))
        
        # Try to connect to each device
        for address in devices:
            try:
                self._log("Connecting to {}...".format(address))
                self.sock = socket.socket(socket.AF_BTH, socket.SOCK_STREAM)
                self.sock.settimeout(5)
                self.sock.connect((address, 1))
                self.connection_active = True
                self.ev3_address = address
                self._log("SUCCESS: Connected to EV3 at {}".format(address))
                return True
            except Exception as e:
                self._log("FAILED: Could not connect to {}: {}".format(address, str(e)))
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
        
        self._log("ERROR: Could not connect to any device")
        return False
    
    def send_command(self, command):
        """Send a single command to EV3"""
        if not self.connection_active or not self.sock:
            self._log("ERROR: No connection to EV3")
            return False
        
        try:
            # Ensure command ends with newline
            if not command.endswith('\n'):
                command = command + '\n'
            
            self._log("Sending: {}".format(command.strip()))
            self.sock.send(command.encode('utf-8'))
            
            # Try to receive acknowledgment (optional - not all EV3 implementations send it back)
            try:
                self.sock.settimeout(2)
                response = self.sock.recv(1024)
                if response:
                    self._log("  → Response: {}".format(response.decode('utf-8', errors='ignore').strip()))
                self.sock.settimeout(None)
            except socket.timeout:
                # No response received (acceptable - command may still execute)
                pass
            except:
                pass
            
            self.commands_sent += 1
            return True
        
        except Exception as e:
            self._log("ERROR: Failed to send command: {}".format(str(e)))
            self.commands_failed += 1
            return False
    
    def load_mission_file(self, filename="commands.txt"):
        """Load mission commands from file"""
        commands = []
        
        # Try multiple paths
        possible_paths = [
            filename,
            os.path.join(os.path.dirname(__file__), "..", "robot", filename),
            os.path.join(os.path.dirname(__file__), "..", filename),
        ]
        
        for path in possible_paths:
            try:
                with open(path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        # Skip empty lines and comments
                        if line and not line.startswith('#'):
                            commands.append(line)
                
                if commands:
                    self._log("Loaded {} commands from {}".format(len(commands), path))
                    return commands
            except OSError:
                continue
        
        self._log("ERROR: Could not find mission file: {}".format(filename))
        return None
    
    def execute_mission(self, mission_file="commands.txt"):
        """Execute a complete mission"""
        self.mission_start_time = time.time()
        
        # Load mission
        commands = self.load_mission_file(mission_file)
        if not commands:
            self._log("ERROR: Could not load mission")
            return False
        
        self._log("Starting mission with {} commands".format(len(commands)))
        self._log("=" * 50)
        
        # Execute each command
        for i, cmd in enumerate(commands, 1):
            self._log("[{}/{}] {}".format(i, len(commands), cmd))
            
            if not self.send_command(cmd):
                self._log("WARNING: Command may have failed")
            
            # Small delay between commands to allow EV3 to process
            time.sleep(0.5)
        
        # Mission complete
        mission_time = time.time() - self.mission_start_time
        self._log("=" * 50)
        self._log("MISSION COMPLETE")
        self._log("Sent: {} commands | Time: {:.1f}s".format(self.commands_sent, mission_time))
        
        return True
    
    def disconnect(self):
        """Close Bluetooth connection"""
        if self.sock:
            try:
                self.sock.close()
                self.connection_active = False
                self._log("Disconnected from EV3")
            except:
                pass


def main():
    """Main entry point"""
    print("=" * 60)
    print("GolfBot 9000: Host Mission Sender")
    print("=" * 60)
    
    # Detect if running on EV3 vs. Host PC
    if is_running_on_ev3():
        print("\n⚠️  ERROR: Running on EV3!\n")
        print("This script is designed to run on the HOST PC (Windows/Mac/Linux),")
        print("NOT on the EV3 robot itself.\n")
        print("Correct Setup:")
        print("  1. On EV3: Run robot/main.py")
        print("  2. On Host PC: Run this script (tools/tools_mission_sender.py)")
        print("  3. Host PC will send commands to EV3 via Bluetooth\n")
        return False
    
    # Create sender
    sender = HostMissionSender()
    
    # Connect to EV3
    if not sender.connect_to_ev3():
        print("ERROR: Could not connect to EV3. Exiting.")
        return False
    
    try:
        # Execute mission
        sender.execute_mission(os.path.join(os.path.dirname(__file__), "..", "robot", "commands.txt"))
    except KeyboardInterrupt:
        print("\n\nMission interrupted by user")
    except Exception as e:
        print("ERROR: {}".format(str(e)))
        return False
    finally:
        sender.disconnect()
    
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
