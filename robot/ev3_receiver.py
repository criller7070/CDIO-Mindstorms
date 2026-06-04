#!/usr/bin/env python3
"""
EV3 Bluetooth receiver.
Receives a mission from the PC and saves it to commands.txt.

Run this on the EV3 first, then press B in the PC nav controller to send.
After receiving, start robot/main.py to execute the mission.
"""
import bluetooth
import os

COMMANDS_FILE = "/home/robot/commands.txt"


def main():
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.bind(("", bluetooth.PORT_ANY))
    sock.listen(1)
    print("Waiting for Bluetooth connection...")

    try:
        client, addr = sock.accept()
        print("Connected: {}".format(addr))

        buf = ""
        collecting = False
        received = []

        while True:
            data = client.recv(1024)
            if not data:
                break
            buf += data.decode("utf-8", errors="ignore")

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                if line == "MISSION_START":
                    collecting = True
                    received = []
                    print("Receiving mission...")
                elif line == "MISSION_END":
                    collecting = False
                    with open(COMMANDS_FILE, "w") as f:
                        for cmd in received:
                            f.write(cmd + "\n")
                    print("Saved {} commands to {}".format(len(received), COMMANDS_FILE))
                    print("Now start robot/main.py to execute.")
                    return
                elif collecting:
                    received.append(line)
                    print("  {}".format(line))
                elif line == "SOUND":
                    os.system("beep")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
