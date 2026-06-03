#!/usr/bin/env python3
"""
Simple camera testing script for Logitech camera.
Tests different camera indices and displays video feed.
Gets actual camera names from Windows device manager.
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import sys
import subprocess
import platform

def get_windows_camera_names():
    """Get actual camera device names from Windows using PowerShell"""
    
    try:
        # Use PowerShell to query all camera-related devices
        ps_command = """
# Try to get Camera devices
$cameras = @()
$cameras += Get-PnpDevice -Class Camera -ErrorAction SilentlyContinue | Select-Object -ExpandProperty FriendlyName
$cameras += Get-PnpDevice -Class 'Image' -ErrorAction SilentlyContinue | Select-Object -ExpandProperty FriendlyName

# Also try to get USB Video devices
Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | 
    Where-Object { $_.FriendlyName -match '(camera|video|webcam)' -and $_.Status -eq 'OK' } | 
    Select-Object -ExpandProperty FriendlyName | 
    ForEach-Object { $cameras += $_ }

# Remove duplicates and output
$cameras | Select-Object -Unique
"""
        result = subprocess.run(
            ['powershell', '-NoProfile', '-Command', ps_command],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0 and result.stdout.strip():
            device_names = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
            return device_names if device_names else None
    except Exception as e:
        pass
    
    # Fallback: Try WMI if PowerShell fails
    try:
        result = subprocess.run(
            ['wmic', 'path', 'win32_pnpentity', 'get', 'name', '/format:list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            device_names = []
            
            for line in lines:
                if '=' in line:
                    device_name = line.split('=', 1)[1].strip()
                    if device_name and any(keyword in device_name.lower() 
                                          for keyword in ['camera', 'video', 'logitech', 'usb', 'webcam']):
                        device_names.append(device_name)
            
            return device_names if device_names else None
    except Exception as e:
        pass
    
    return None


def test_single_camera(camera_index):
    """Test a single camera index"""
    print(f"\nTesting camera index {camera_index}...", end=" ")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print("❌ FAILED - Camera not found")
        return False, None
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"✓ OPENED", end="")
    print(f" | Res: {width}x{height} | FPS: {fps}", end="")
    
    # Try to read frames - this is the real test
    frame_success = False
    for attempt in range(3):
        ret, frame = cap.read()
        if ret:
            frame_success = True
            break
    
    if frame_success:
        print(f" | Frame read: ✓")
    else:
        print(f" | Frame read: ❌ (can't actually read)")
    
    cap.release()
    return frame_success, (width, height, fps) if frame_success else None


def test_all_cameras():
    """Test camera indices 0-15 to find all working cameras"""
    print("=" * 60)
    print("SCANNING FOR AVAILABLE CAMERAS (indices 0-15)")
    print("=" * 60)
    
    # Get actual camera names from Windows
    print("\nQuerying Windows for camera devices...")
    all_device_names = get_windows_camera_names()
    
    # Filter to only actual camera devices (exclude DFU, Microphone, etc.)
    device_names = []
    excluded_keywords = ['dfu', 'microphone', 'audio', 'sound']
    
    if all_device_names:
        device_names = [
            name for name in all_device_names 
            if not any(keyword in name.lower() for keyword in excluded_keywords)
        ]
        
        print(f"Found {len(all_device_names)} device(s) in Windows:")
        for i, name in enumerate(all_device_names):
            is_camera = name not in device_names
            status = " ❌ (filtered)" if is_camera else " ✓ (camera)"
            print(f"  • {name}{status}")
        
        print(f"\nCamera devices detected in Windows:")
        for i, name in enumerate(device_names):
            is_logitech = "logitech" in name.lower()
            marker = " ⚠️ LOGITECH" if is_logitech else ""
            print(f"  {i+1}. {name}{marker}")
    else:
        print("Could not query Windows devices. Testing camera indices directly...\n")
    
    found_cameras = []
    camera_props = {}  # Store camera properties for naming
    
    print("\n" + "Testing indices 0-15...")
    for i in range(16):
        success, props = test_single_camera(i)
        if success:
            found_cameras.append(i)
            camera_props[i] = props
    
    # Show results
    print("\n" + "=" * 60)
    if found_cameras:
        print(f"✓ Found {len(found_cameras)} working camera index(es):\n")
        for idx in found_cameras:
            props = camera_props[idx]
            width, height, fps = props if props else (0, 0, 0)
            is_high_res = width >= 1920 and height >= 1080
            marker = "🎥 HIGH RES (Possibly Logitech?)" if is_high_res else ""
            print(f"  Index {idx:2d}: {width}x{height} @ {fps}fps {marker}")
        
        # Highlight high-res cameras
        high_res_indices = [idx for idx in found_cameras 
                           if camera_props[idx][0] >= 1920 and camera_props[idx][1] >= 1080]
        
        if high_res_indices:
            print(f"\n💡 HIGH RESOLUTION CAMERAS (likely to be USB cameras like Logitech):")
            for idx in high_res_indices:
                props = camera_props[idx]
                print(f"  Index {idx}: {props[0]}x{props[1]} @ {props[2]}fps - TEST THIS FIRST!")
        
        # If we have Windows device names, show them separately
        if device_names:
            print(f"\n⚠️  Windows lists {len(device_names)} camera device(s):")
            for i, name in enumerate(device_names):
                print(f"  {i+1}. {name}")
            
            logitech_count = sum(1 for name in device_names if 'logitech' in name.lower())
            if logitech_count > 0:
                print(f"\n🎥 Found {logitech_count} Logitech device(s) in Windows!")
                print(f"✓ Windows Camera app can see it, so it DOES exist")
                if high_res_indices:
                    print(f"✓ Found high-res OpenCV indices: {high_res_indices}")
                    print(f"→ Your Logitech camera is likely at index {high_res_indices[0] if high_res_indices else '???'}")
                else:
                    print(f"⚠️  But no high-res indices found - may need to test manually")
    else:
        print("❌ No cameras found!")
    print("=" * 60)
    
    return found_cameras, camera_props


def run_camera_feed(camera_index, camera_name="Camera"):
    """Run live camera feed for testing"""
    print(f"\nOpening {camera_name} (index {camera_index})...")
    print("Press 'q' to quit, 's' to save screenshot\n")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ Cannot open camera {camera_index}")
        return False
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("❌ Failed to read frame")
                break
            
            frame_count += 1
            
            # Add info overlay
            info_text = f"{camera_name} (Index {camera_index}) | Frame {frame_count}"
            cv2.putText(frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            fps = cap.get(cv2.CAP_PROP_FPS)
            h, w = frame.shape[:2]
            res_text = f"Resolution: {w}x{h} | FPS: {int(fps)}"
            cv2.putText(frame, res_text, (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow(f'{camera_name} (Index {camera_index})', frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nExiting camera feed...")
                break
            elif key == ord('s'):
                filename = f"camera_{camera_index}_screenshot.png"
                cv2.imwrite(filename, frame)
                print(f"✓ Screenshot saved: {filename}")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"Captured {frame_count} frames")
        return True


def main():
    print("\n╔════════════════════════════════════════════════════════╗")
    print("║          LEGO Mindstorms Camera Test Tool              ║")
    print("╚════════════════════════════════════════════════════════╝\n")
    
    # First, scan for cameras
    found_cameras, camera_props = test_all_cameras()
    
    if not found_cameras:
        return
    
    # Ask user which camera to test
    print("\nOptions:")
    print("1. Test camera feed (press 'q' to quit)")
    print("2. Run detailed diagnostic")
    print("3. Exit")
    
    choice = input("\nEnter choice (1-3): ").strip()
    
    if choice == "1":
        print(f"\nAvailable camera indices:")
        for idx in found_cameras:
            props = camera_props.get(idx, (0, 0, 0))
            print(f"  Index {idx}: {props[0]}x{props[1]} @ {props[2]}fps")
        camera_idx = input(f"Enter camera index to test (default {found_cameras[0]}): ").strip()
        camera_idx = int(camera_idx) if camera_idx else found_cameras[0]
        print(f"\nStarting camera feed for index {camera_idx}...")
        run_camera_feed(camera_idx, f"Camera Index {camera_idx}")
    
    elif choice == "2":
        print(f"\nAvailable camera indices:")
        for idx in found_cameras:
            props = camera_props.get(idx, (0, 0, 0))
            print(f"  Index {idx}: {props[0]}x{props[1]} @ {props[2]}fps")
        camera_idx = input(f"Enter camera index (default {found_cameras[0]}): ").strip()
        camera_idx = int(camera_idx) if camera_idx else found_cameras[0]
        run_detailed_diagnostic(camera_idx)
    
    else:
        print("Exiting...")


def run_detailed_diagnostic(camera_index):
    """Run detailed camera diagnostics"""
    print(f"\n{'='*50}")
    print(f"DETAILED DIAGNOSTIC FOR CAMERA {camera_index}")
    print(f"{'='*50}\n")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ Cannot open camera {camera_index}")
        return
    
    # List all available properties
    properties = {
        cv2.CAP_PROP_FRAME_WIDTH: "Frame Width",
        cv2.CAP_PROP_FRAME_HEIGHT: "Frame Height",
        cv2.CAP_PROP_FPS: "FPS",
        cv2.CAP_PROP_BRIGHTNESS: "Brightness",
        cv2.CAP_PROP_CONTRAST: "Contrast",
        cv2.CAP_PROP_SATURATION: "Saturation",
        cv2.CAP_PROP_EXPOSURE: "Exposure",
        cv2.CAP_PROP_GAIN: "Gain",
        cv2.CAP_PROP_AUTOFOCUS: "Autofocus",
    }
    
    print("Camera Properties:")
    print("-" * 50)
    
    for prop_id, prop_name in properties.items():
        try:
            value = cap.get(prop_id)
            print(f"  {prop_name:.<40} {value}")
        except:
            print(f"  {prop_name:.<40} N/A")
    
    # Test frame reading
    print("\n" + "-" * 50)
    print("Frame Reading Test:")
    print("-" * 50)
    
    success_count = 0
    fail_count = 0
    shapes = []
    
    for i in range(10):
        ret, frame = cap.read()
        if ret:
            success_count += 1
            shapes.append(frame.shape)
        else:
            fail_count += 1
    
    print(f"  Successfully read: {success_count}/10 frames")
    print(f"  Failed reads: {fail_count}/10 frames")
    
    if shapes:
        print(f"  Frame shape: {shapes[0]}")
        print(f"  Consistent: {'✓ Yes' if len(set(shapes)) == 1 else '❌ No (variable sizes)'}")
    
    cap.release()
    
    print("\n" + "=" * 50)
    if success_count >= 9:
        print("✓ Camera test PASSED - Ready to use!")
    else:
        print("❌ Camera test FAILED - Check connection")
    print("=" * 50)


if __name__ == "__main__":
    main()
