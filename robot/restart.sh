#!/bin/bash
# Restart script for EV3 follow-mode. Called by loop.py --profile/--tcp.
#
# Kill only our own processes: pybricks-micropython and ev3_server.
# Let brickrun exit naturally when its child (pybricks) dies.
# Do NOT touch brickd, brickman, or other system processes.

# Kill pybricks-micropython (our child process)
PY_PID=$(ps -e -o pid,comm | awk '/pybricks-microp/{print $1}')
[ -n "$PY_PID" ] && kill "$PY_PID" 2>/dev/null

# Kill ev3_server (TCP bridge) via saved PID
kill $(cat /tmp/ev3_server.pid 2>/dev/null) 2>/dev/null

# Give brickrun 3s to exit after its child dies
sleep 3

# Reset IPC to known-clean state
echo "0 STOP" > /home/robot/cl_cmd.txt
echo "0 DONE" > /home/robot/cl_ack.txt

# Clear old log so stale "Follow loop ready" is not detected on restart
rm -f /tmp/main.log

cd /home/robot/CDIO-Mindstorms/robot

# Start TCP bridge
nohup python3 ev3_server.py --tcp </dev/null >/tmp/ev3_server.log 2>&1 &
echo $! > /tmp/ev3_server.pid
sleep 2

# Start pybricks (no -r: avoid hardware reset which can destabilize the brick)
nohup brickrun -r -- pybricks-micropython main.py --follow </dev/null >/tmp/main.log 2>&1 &
echo "restart done"
