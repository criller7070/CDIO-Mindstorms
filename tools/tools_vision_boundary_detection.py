#!/usr/bin/env python3
"""
Entry point — kept so existing scripts/docs that reference this filename still work.
The actual implementation lives in:
  vision_config.py       – constants & colour-range persistence
  vision_detector.py     – ball & field detection engine
  vision_calibration.py  – live HSV trackbar calibration
  vision_app.py          – interactive planning menu (main loop)
"""
from vision_app import VisionApp

if __name__ == "__main__":
    VisionApp().run()
