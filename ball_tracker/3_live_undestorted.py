import cv2
import numpy as np
import time

# Load calibration data
try:
    with np.load('cam_calib.npz') as data:
        mtx = data['mtx']
        dist = data['dist']
        print("Calibration loaded successfully.")
except:
    print("Error: 'cam_calib.npz' not found. Run Step 2 first.")
    exit()

def gstreamer_pipeline():
    return (
        "libcamerasrc ! "
        "video/x-raw, width=640, height=480, framerate=60/1, format=NV12 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true sync=false"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

# --- PRE-CALCULATE MAPS (Optimization) ---
h, w = 480, 640
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
# -----------------------------------------

prev_time = 0

print("Running Un-Distorted Stream @ 60 FPS target...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- THE MAGIC STEP (Fast Remapping) ---
    dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    
    # Optional: Crop the image to remove black curved edges
    # x, y, w, h = roi
    # dst = dst[y:y+h, x:x+w]
    # ---------------------------------------

    # FPS Calc
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    cv2.putText(dst, f"FPS: {int(fps)}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Calibrated View", dst)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
