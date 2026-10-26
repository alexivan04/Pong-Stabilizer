import cv2
import numpy as np
import time

with np.load('cam_calib_fisheye.npz') as data:
    K = data['K']
    D = data['D']

def gstreamer_pipeline():
    return (
        "libcamerasrc ! "
        "video/x-raw, width=640, height=480, framerate=60/1, format=NV12 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true sync=false"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

h, w = 960, 1280

# optional: balance controls cropping vs preserving FOV
balance = 1.0  # 0 = crop more, 1 = keep full FOV

new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K, D, (w, h), np.eye(3), balance=balance
)

map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2
)

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    cv2.putText(undistorted, f"FPS: {int(fps)}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    cv2.imshow("Fisheye Corrected", undistorted)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
