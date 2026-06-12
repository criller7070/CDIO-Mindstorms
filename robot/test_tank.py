#!/usr/bin/env python3
"""
Test script for ev3dev2 MoveTank API.
Motors: left=OUTPUT_A, right=OUTPUT_B

Run on the EV3: python3 test_tank.py
"""
from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.sound import Sound
import time

sound = Sound()
tank = MoveTank(OUTPUT_A, OUTPUT_B)

sound.speak("tight pussy girl")
time.sleep(1)

# --- Forward 2 rotations ---
print("Forward 2 rotations at 50%")
tank.on_for_rotations(SpeedPercent(50), SpeedPercent(50), 2)
time.sleep(0.5)

# --- Turn right: left motor moves, right stays ---
print("Turning right 1 rotation")
tank.on_for_rotations(SpeedPercent(100), SpeedPercent(-100), 100)
time.sleep(0.5)

# --- Reverse 2 rotations ---
print("Reverse 2 rotations at 40%")
tank.on_for_rotations(SpeedPercent(-40), SpeedPercent(-40), 2)
time.sleep(0.5)

# --- Stop ---
tank.off()
print("Done")
sound.speak("Done")
