#!/usr/bin/env python3
"""
Run this TCP or bluetooth server on the EV3, then loop.py on your own machine
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
