#!/usr/bin/env python3
"""
EV3 Bluetooth bridge for CLOSED-LOOP control.

Runs under ev3dev's python3 (PyBluez), the same runtime as the old
ev3_receiver.py.  It keeps a persistent Bluetooth connection to the PC and,
for every command line it receives, hands that single command to the pybricks
executor (robot/main.py --follow) through two tiny files and returns the
executor's DONE acknowledgement to the PC.

Why two processes?  Motor control here uses pybricks-micropython, while
Bluetooth uses python3/PyBluez - two different interpreters that can't share a
process.  They cooperate through a sequence-numbered file bridge:

    cl_cmd.txt : "<seq> <COMMAND>"   written here, read by the executor
    cl_ack.txt : "<seq> DONE"        written by the executor, read here

Start BOTH on the EV3 (order doesn't matter):

    python3 robot/ev3_server.py --tcp        # TCP/WiFi  (default port 9999)
    python3 robot/ev3_server.py              # Bluetooth (RFCOMM)
    brickrun -- pybricks-micropython robot/main.py --follow

Then run host/loop.py on the PC (use --tcp <robot-ip> or --profile <name> to match).
"""
import os
import sys
import time

CMD_FILE = "/home/robot/cl_cmd.txt"
ACK_FILE = "/home/robot/cl_ack.txt"

DEFAULT_TCP_PORT = 9999
ACK_TIMEOUT = 25.0   # seconds to wait for the executor before giving up
POLL = 0.02


def write_atomic(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def read_seq(path):
    """first whitespace-delimited int in path, or -1 if unreadable."""
    try:
        with open(path, "r") as f:
            return int(f.readline().strip().split(" ", 1)[0])
    except (OSError, ValueError, IndexError):
        return -1


def wait_for_ack(seq, timeout=ACK_TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if read_seq(ACK_FILE) == seq:
            return True
        time.sleep(POLL)
    return False


def serve(client, seq):
    """handle one PC connection. seq must keep increasing across reconnects -
    restarting at 1 could collide with what the executor already processed.
    returns the updated counter."""
    buf = ""
    while True:
        data = client.recv(1024)
        if not data:
            print("PC closed the connection.")
            return seq
        buf += data.decode("utf-8", errors="ignore")

        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            cmd = line.strip()
            if not cmd:
                continue
            if cmd == "PING":
                client.send(b"PONG\n")
                continue

            seq += 1
            write_atomic(CMD_FILE, "{} {}\n".format(seq, cmd))
            ok = wait_for_ack(seq)
            client.send(b"DONE\n" if ok else b"TIMEOUT\n")
            print("  [{}] {} -> {}".format(seq, cmd, "DONE" if ok else "TIMEOUT"))


def make_tcp_server(port):
    import socket
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", port))
    srv.listen(1)
    print("Closed-loop bridge (TCP) listening on port {}. Waiting for PC...".format(port))
    return srv


def make_bluetooth_server():
    import bluetooth
    srv = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    srv.bind(("", bluetooth.PORT_ANY))
    srv.listen(1)
    print("Closed-loop bridge (Bluetooth) listening. Waiting for PC...")
    return srv


def main():
    # resume counter from disk so a restart never reuses a seq the executor
    # already processed. only seed files when missing - don't clobber a live ack.
    seq = read_seq(CMD_FILE)
    if seq < 0:
        seq = 0
        write_atomic(CMD_FILE, "0 INIT\n")
    if read_seq(ACK_FILE) < 0:
        write_atomic(ACK_FILE, "0 INIT\n")

    if "--tcp" in sys.argv:
        i = sys.argv.index("--tcp")
        port = int(sys.argv[i + 1]) if i + 1 < len(sys.argv) and sys.argv[i + 1].isdigit() \
            else DEFAULT_TCP_PORT
        server = make_tcp_server(port)
    else:
        server = make_bluetooth_server()

    try:
        while True:
            client, addr = server.accept()
            print("Connected: {}".format(addr))
            try:
                seq = serve(client, seq)
            except OSError as e:
                print("Connection error: {}".format(e))
            finally:
                try:
                    client.close()
                except OSError:
                    pass
            print("Waiting for a new PC connection...")
    finally:
        server.close()


if __name__ == "__main__":
    main()
