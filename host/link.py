#!/usr/bin/env python3
"""
Transport links — all expose send_and_wait(cmd) -> bool.

BluetoothLink  legacy Bluetooth via tools_mission_sender
TCPLink        WiFi over TCP (primary, matches ev3_server.py --tcp)
SimLink        virtual robot for offline loop testing
"""
import math
import time
import numpy as np

from config import FORWARD_CMD_SCALE


class BluetoothLink:
    """Sends single commands to the EV3 bridge and blocks for its DONE ack."""

    def __init__(self, ev3_address=None):
        import sys, os
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from tools.tools_mission_sender import HostMissionSender
        self.sender = HostMissionSender(ev3_address)

    def connect(self):
        if not self.sender.connect_to_ev3(self.sender.ev3_address):
            return False
        return self.sender.send_and_wait("PING", timeout=5.0, ack_token="PONG")

    def send_and_wait(self, command):
        return self.sender.send_and_wait(command)

    def close(self):
        self.sender.disconnect()


class TCPLink:
    """Sends single commands to the EV3 bridge over TCP/WiFi and blocks for DONE.

    Lower latency than Bluetooth. Matches robot/ev3_server.py --tcp.
    """

    def __init__(self, host, port=9999, timeout=20.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        import socket
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=10.0)
        except OSError as e:
            print("TCP connect to {}:{} failed: {}".format(self.host, self.port, e))
            return False
        return self._send_wait("PING\n", "PONG")

    def send_and_wait(self, command):
        if not command.endswith("\n"):
            command += "\n"
        return self._send_wait(command, "DONE")

    def _send_wait(self, payload, token):
        import socket
        try:
            self.sock.sendall(payload.encode("utf-8"))
        except OSError as e:
            print("TCP send failed: {}".format(e))
            return False
        self.sock.settimeout(self.timeout)
        buf = ""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            try:
                data = self.sock.recv(1024)
            except socket.timeout:
                break
            if not data:
                print("TCP connection closed by robot.")
                return False
            buf += data.decode("utf-8", errors="ignore")
            if token in buf:
                return True
            if "TIMEOUT" in buf:
                print("Robot reported TIMEOUT on: {}".format(payload.strip()))
                return False
        print("TCP timeout waiting for {} (cmd: {})".format(token, payload.strip()))
        return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass


class SimLink:
    """Virtual robot for testing the loop with no hardware.

    Mirrors robot/main.py command semantics: FORWARD:v moves v / FORWARD_CMD_SCALE
    physical mm, and TURN coasts a constant turn_overshoot_deg past the commanded
    angle. Optional gains/noise add error to prove the closed loop corrects drift.
    """

    def __init__(self, pose, px_per_mm, turn_gain=1.0, fwd_gain=1.0,
                 lateral_noise_mm=0.0, turn_overshoot_deg=0.0, seed=0):
        self.x, self.y, self.heading = pose
        self.px_per_mm = px_per_mm
        self.turn_gain = turn_gain
        self.fwd_gain = fwd_gain
        self.lateral_noise_mm = lateral_noise_mm
        self.turn_overshoot_deg = turn_overshoot_deg
        self.rng = np.random.default_rng(seed)
        self.trail = [(self.x, self.y)]

    def send_and_wait(self, command):
        cmd, _, val = command.partition(":")
        cmd = cmd.strip().upper()
        if cmd == "TURN":
            v = float(val)
            applied = v * self.turn_gain
            if v != 0:
                applied += math.copysign(self.turn_overshoot_deg, v)
            self.heading += applied
            self.heading = (self.heading + 180.0) % 360.0 - 180.0
        elif cmd == "FORWARD":
            d_mm = (float(val) / FORWARD_CMD_SCALE) * self.fwd_gain
            d_px = d_mm * self.px_per_mm
            hr = math.radians(self.heading)
            self.x += d_px * math.cos(hr)
            self.y += d_px * math.sin(hr)
            if self.lateral_noise_mm:
                lat = self.rng.normal(0.0, self.lateral_noise_mm) * self.px_per_mm
                self.x += lat * math.cos(hr + math.pi / 2)
                self.y += lat * math.sin(hr + math.pi / 2)
            self.trail.append((self.x, self.y))
        return True

    def pose(self):
        return (self.x, self.y, self.heading)
