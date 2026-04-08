# Vision-Based Boundary Detection for EV3 Robot

## Overview

This system uses your **Logitech camera** as a "vision system" for your EV3 robot to autonomously navigate within course boundaries without hitting walls or obstacles.

### How It Works:

```
┌─────────────────────────┐
│   Logitech Camera       │  <- Streams video
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────────────────────────┐
│  Host PC: vision_boundary_detection.py      │
│  • Detects walls/obstacles via OpenCV       │
│  • Calculates steering commands             │
│  • Sends commands via Bluetooth             │
└────────────┬────────────────────────────────┘
             │ (Bluetooth)
             ▼
┌─────────────────────────┐
│   EV3 Robot             │
│   Executes commands     │
│   Moves accordingly     │
└─────────────────────────┘
```

---

## Setup Instructions

### Step 1: Install Required Libraries on Host PC

```bash
# OpenCV for computer vision
pip install opencv-python

# NumPy for image processing  
pip install numpy

# That's it! No additional Bluetooth library needed on Windows.
# Windows socket API is built-in (socket.AF_BTH)
```

**Why no PyBluez?** PyBluez has compatibility issues with modern Python on Windows. Instead, this system uses Windows' native Bluetooth socket API directly, which is more reliable.

### Step 2: Prepare Your Logitech Camera

1. **Connect the camera** to your host PC via USB
2. **Test the camera** (find which device index):
   ```bash
   # Create test_camera.py:
   import cv2
   cap = cv2.VideoCapture(0)
   if cap.isOpened():
       ret, frame = cap.read()
       print("Camera ready (index 0)")
   else:
       cap = cv2.VideoCapture(1)  # Try index 1
       print("Camera ready (index 1)")
   ```

3. **Update camera index** in `vision_boundary_detection.py` if needed (default is 0)

### Step 3: Set Up EV3

1. **Upload firmware** with PyBricks support
2. **Copy `ev3_nav_controller.py`** to EV3 and name it `main.py` or run it directly
3. **Enable Bluetooth** on EV3
4. **Pair EV3 with your host PC**:
   - Windows: Settings → Bluetooth & devices → Add device
   - Select your EV3 from the list
   - Use default PIN (usually 1234)

### Step 4: Run the System

#### On EV3:
```bash
# Run the navigation controller (listens for Bluetooth commands)
# It will say "Navigation ready" when started
# Then "Waiting for connection..." until PC connects
```

#### On Host PC:
```bash
# Run the vision system
python vision_boundary_detection.py
```

---

## Configuration & Calibration

### Color Calibration (Most Important!)

The system detects walls based on color. You need to define what colors your walls are.

Edit `vision_boundary_detection.py` around line 45-58:

```python
# Default: Black/dark walls (BGR format - note it's BGR not RGB!)
self.lower_wall = np.array([0, 0, 0])
self.upper_wall = np.array([100, 100, 100])

# Red boundaries
self.lower_red = np.array([0, 0, 100])
self.upper_red = np.array([50, 50, 255])

# Blue boundaries
self.lower_blue = np.array([100, 0, 0])
self.upper_blue = np.array([255, 50, 50])
```

**How to find your color ranges:**

1. Run the script with your course visible
2. Look at the "Mask" window - walls should be white, everything else black
3. If walls aren't showing, adjust the color ranges
4. Use an HSV color picker tool to find your exact wall colors

Example color ranges (BGR):
- **Black walls**: `[0,0,0]` to `[100,100,100]`
- **White walls**: `[150,150,150]` to `[255,255,255]`
- **Gray walls**: `[80,80,80]` to `[180,180,180]`

### Detection Parameters

```python
self.wall_threshold = 50          # Distance to wall (pixels) - decrease for more sensitivity
self.min_contour_area = 500       # Minimum wall size to detect - decrease to detect smaller walls
```

### Robot Parameters

Edit `ev3_nav_controller.py` around line 21-24:

```python
self.forward_speed = 200    # mm/s (increase for faster movement)
self.turn_speed = 90        # deg/s
self.turn_angle = 45        # degrees to turn when avoiding
```

---

## Commands Sent to EV3

The host PC sends these commands via Bluetooth:

| Command | Action |
|---------|--------|
| `FORWARD` | Drive forward at set speed |
| `STOP` | Stop immediately |
| `TURN_LEFT` | Turn left by 45° |
| `TURN_RIGHT` | Turn right by 45° |
| `REVERSE` | Move backward |
| `SPEED:300` | Set speed to 300 mm/s |

---

## Visualization Window

When running `vision_boundary_detection.py`, you'll see:

### Main Window (Boundary Detection):
- **Green vertical lines**: Left (1/3) and right (2/3) border lines
- **Yellow horizontal line**: Front detection zone
- **Red circles with boxes**: Detected obstacles/walls
- **Green status**: Shows "CONNECTED" or "DISCONNECTED"

### Mask Window:
- **White areas**: Detected walls/obstacles
- **Black areas**: Clear path
- Helps debug color detection

### Console Output:
```
Frame 100: FORWARD | Obstacles: L=False C=False R=False F=False
Frame 105: TURN_RIGHT | Obstacles: L=True C=False R=False F=False
```

---

## Troubleshooting

### Problem: "Cannot open camera"
**Solution:**
- Check camera is connected and not in use by another app
- Try changing `camera_index` to 1 or 2
- Test with another Python script:
  ```python
  import cv2
  cap = cv2.VideoCapture(0)
  print(cap.isOpened())  # Should print True
  ```

### Problem: "No EV3 found"
**Solution:**
- Ensure EV3 is paired on Windows
- Try manually entering EV3 address:
  ```python
  detector.connect_to_ev3("00:16:53:xx:xx:xx")  # Your EV3 MAC address
  ```
- Check Windows Bluetooth settings for device address

### Problem: Walls not being detected (blank mask window)
**Solution:**
- Adjust color ranges in `self.lower_wall` and `self.upper_wall`
- Increase `self.min_contour_area` to larger value
- Check lighting - camera needs good illumination
- Print HSV values to debug:
  ```python
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  print(cv2.mean(hsv))  # Shows average HSV of frame
  ```

### Problem: Robot not stopping at walls
**Solution:**
- Decrease `self.wall_threshold` (currently 50 pixels)
- Decrease `self.forward_speed` for slower, more controllable movement
- Increase `self.min_contour_area` to focus on larger obstacles only
- Test color detection in Mask window first

### Problem: Robot spinning in circles
**Solution:**
- Increase `self.turn_angle` (currently 45°)
- Decrease forward speed for better control
- Adjust `self.turn_speed` for more controlled turns
- Place larger, clearer boundary markers

### Problem: "ModuleNotFoundError: cv2" or "numpy"
**Solution:**
```bash
# Reinstall OpenCV
pip install --upgrade opencv-python

# Reinstall NumPy
pip install --upgrade numpy
```

**Note:** You should NOT need to install any Bluetooth library on Windows - the system uses built-in Windows socket API (`socket.AF_BTH`).

---

## Advanced Features

### 1. **Fine-Tuning Detection Quality**

Create a calibration script to find exact color values:

```python
import cv2
import numpy as np

def color_picker():
    cap = cv2.VideoCapture(0)
    
    def callback(x):
        pass
    
    cv2.namedWindow('image')
    cv2.createTrackbar('H_low', 'image', 0, 180, callback)
    cv2.createTrackbar('H_high', 'image', 180, 180, callback)
    cv2.createTrackbar('S_low', 'image', 0, 255, callback)
    cv2.createTrackbar('S_high', 'image', 255, 255, callback)
    
    while True:
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        h_low = cv2.getTrackbarPos('H_low', 'image')
        h_high = cv2.getTrackbarPos('H_high', 'image')
        s_low = cv2.getTrackbarPos('S_low', 'image')
        s_high = cv2.getTrackbarPos('S_high', 'image')
        
        mask = cv2.inRange(hsv, (h_low, s_low, 0), (h_high, s_high, 255))
        
        cv2.imshow('image', mask)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        print(f"H: {h_low}-{h_high}, S: {s_low}-{s_high}")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    color_picker()
```

### 2. **Different Wall Types in Same Course**

Combine multiple mask ranges:

```python
# In detect_obstacles method:
mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv, lower_blue1, upper_blue1)
mask3 = cv2.inRange(hsv, lower_black1, upper_black1)

# Combine all wall types
mask = cv2.bitwise_or(mask1, mask2)
mask = cv2.bitwise_or(mask, mask3)
```

### 3. **Recording Run for Debugging**

```python
# Add to run() method in BoundaryDetector class:
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('navigation_run.mp4', fourcc, 30.0, (640, 480))

# In main loop:
out.write(frame)

# In cleanup:
out.release()
```

### 4. **Adjustable Speed Based on Wall Distance**

```python
def calculate_steering_command(self, obstacle_info):
    """Advanced: adjust speed based on obstacle proximity"""
    if obstacle_info['obstacles']:
        min_distance = min(obs['width'] for obs in obstacle_info['obstacles'])
        if min_distance < 100:
            return "SLOW"  # New command for slow speed
    return self.calculate_normal_command(obstacle_info)
```

---

## Performance Tips

1. **Reduce frame resolution** if laggy: `self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)`
2. **Reduce FPS** if CPU-bound: `self.cap.set(cv2.CAP_PROP_FPS, 15)`
3. **Skip frames** for faster processing:
   ```python
   if frame_count % 5 == 0:  # Process every 5th frame
       obstacle_info, mask = self.detect_obstacles(frame)
   ```
4. **Use threading** for Bluetooth to avoid blocking:
   ```python
   import threading
   thread = threading.Thread(target=self.send_command, args=(cmd,))
   thread.daemon = True
   thread.start()
   ```

---

## Testing Checklist

- [ ] Camera connects and shows video
- [ ] EV3 pairs with host PC
- [ ] EV3 script starts (says "Navigation ready")
- [ ] Host script finds EV3 (prints "Connected to EV3")
- [ ] Walls appear as white in Mask window
- [ ] Console shows commands being sent
- [ ] EV3 receives commands (screen updates, makes sounds)
- [ ] Robot turns away from walls
- [ ] Robot stops when approaching obstacles

---

## Quick Start Command

```bash
# 1. Install dependencies (only 2!)
pip install opencv-python numpy

# Terminal 1: Start EV3 script (on the robot)
python ev3_nav_controller.py

# Terminal 2: Start vision system (on host PC)
python vision_boundary_detection.py

# Press 'q' to exit vision system
```

---

## References

- **OpenCV Color Detection**: https://docs.opencv.org/4.5.2/df/df6/classcv_1_1Moments.html
- **HSV Color Space**: https://en.wikipedia.org/wiki/HSL_and_HSV
- **PyBricks**: https://pybricks.com/
- **PyBluez**: https://github.com/pybluez/pybluez

---

*This system gives your EV3 robot 360° awareness using computer vision!*
