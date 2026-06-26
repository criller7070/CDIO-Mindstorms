#!/bin/bash
# restart script; called over SSH by loop.py when it needs a mid-session restart.
# THIS IS NOT THE ENTRY-POINT! run ev3_server.py on robot, then loop.py on host PC
#
# kills only our processes (pybricks-micropython + ev3_server).
# brickrun exits naturally once its child is gone.
# don't touch brickd, brickman, or other system daemons.

# 1. kill pybricks
PY_PID=$(ps -e -o pid,comm | awk '/pybricks-microp/{print $1}')
[ -n "$PY_PID" ] && kill "$PY_PID" 2>/dev/null

# 2. kill ev3_server via saved pid
kill $(cat /tmp/ev3_server.pid 2>/dev/null) 2>/dev/null

# wait for pybricks to fully exit before brickrun starts a new one;
# on a loaded EV3 the process may still hold port/gyro locks during kernel teardown.
for i in $(seq 1 20); do
    ps -e -o comm | grep -q pybricks-microp || break
    sleep 0.5
done

# give brickrun a moment to exit after its child is gone
sleep 1

# 3. reset IPC to a known-clean state
echo "0 STOP" > /home/robot/cl_cmd.txt
echo "0 DONE" > /home/robot/cl_ack.txt
rm -f /tmp/main.log  # wipe stale log so "Follow loop ready" isn't detected on restart

cd /home/robot/CDIO-Mindstorms/robot

# 4. start TCP bridge
nohup python3 ev3_server.py --tcp </dev/null >/tmp/ev3_server.log 2>&1 &
echo $! > /tmp/ev3_server.pid
sleep 2

# 5. start pybricks
nohup brickrun -r -- pybricks-micropython main.py --follow </dev/null >/tmp/main.log 2>&1 &
echo "restart done"
