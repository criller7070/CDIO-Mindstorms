import cv2, time, os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
cap = cv2.VideoCapture(1)
time.sleep(1.5)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
p = cv2.aruco.DetectorParameters()
det = cv2.aruco.ArucoDetector(d, p)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Camera - press Q to quit", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
