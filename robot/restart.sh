#!/bin/bash
# Restart script for EV3 mid loop. Runs on the brick itself.
#
# Works by killing only our own processes, that being pybricks-micropython
# and ev3_server, and then letting the brick kill its children naturally;
# this means DO NOT touch brickd, brickman, or other system processes.

# 1. Kill pybricks-micropython (our child process)
PY_PID=$(ps -e -o pid,comm | awk '/pybricks-microp/{print $1}')
[ -n "$PY_PID" ] && kill "$PY_PID" 2>/dev/null

# 2. Kill ev3_server (TCP bridge) via saved PID
kill $(cat /tmp/ev3_server.pid 2>/dev/null) 2>/dev/null
sleep 3

# 3. Reset IPC to known-clean state
echo "0 STOP" > /home/robot/cl_cmd.txt
echo "0 DONE" > /home/robot/cl_ack.txt
rm -f /tmp/main.log # clear ol logs

cd /home/robot/CDIO-Mindstorms/robot

# 4. Start TCP server
nohup python3 ev3_server.py --tcp </dev/null >/tmp/ev3_server.log 2>&1 &
echo $! > /tmp/ev3_server.pid
sleep 2

# 5. Start pybricks
nohup brickrun -r -- pybricks-micropython main.py --follow </dev/null >/tmp/main.log 2>&1 &
echo "restart done"
