"""
Simple test to make EV3 produce a sound via Bluetooth.
"""

from ev3_communication import EV3Controller

try:
    # Connect to EV3
    print("Connecting to EV3...")
    ev3 = EV3Controller(ev3_name='EV3')
    
    # Send sound command
    print("Sending sound command...")
    ev3.send_command("SOUND\n")
    
    print("Done!")
    
except Exception as e:
    print(f"Error: {e}")
finally:
    ev3.close()
