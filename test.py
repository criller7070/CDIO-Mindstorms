import socket
import subprocess
import re

# Find EV3 address from paired devices
def find_ev3():
    try:
        result = subprocess.run(
            ["reg", "query", "HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Services\\BTHPORT\\Parameters\\Devices"],
            capture_output=True,
            text=True
        )
        addresses = re.findall(r'[0-9A-Fa-f]{12}', result.stdout)
        if addresses:
            print("Found Bluetooth devices:")
            formatted = []
            for addr in addresses:
                fmt = ':'.join([addr[i:i+2] for i in range(0, 12, 2)])
                print(f"  {fmt}")
                formatted.append(fmt)
            return formatted
    except:
        pass
    
    return []

addresses = find_ev3()

if addresses:
    print("\nTrying to connect...")
    for address in addresses:
        try:
            print(f"Trying {address}...", end=" ")
            sock = socket.socket(socket.AF_BTH, socket.SOCK_STREAM)
            sock.connect((address, 1))
            sock.send(b"SOUND")
            print("Connected! Sent SOUND")
            sock.close()
            break
        except Exception as e:
            print(f"Failed")
else:
    print("No devices found")
