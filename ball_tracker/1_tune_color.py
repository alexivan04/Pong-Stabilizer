import cv2
import numpy as np

def nothing(x): pass

cv2.namedWindow("Tuner")
cv2.createTrackbar("L - H", "Tuner", 20, 179, nothing)
cv2.createTrackbar("L - S", "Tuner", 100, 255, nothing)
cv2.createTrackbar("L - V", "Tuner", 100, 255, nothing)
cv2.createTrackbar("U - H", "Tuner", 40, 179, nothing)
cv2.createTrackbar("U - S", "Tuner", 255, 255, nothing)
cv2.createTrackbar("U - V", "Tuner", 255, 255, nothing)

# Optimized Pipeline
pipeline = (
    "libcamerasrc ! video/x-raw, width=640, height=480, framerate=60/1, format=NV12 ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if not ret: break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L - H", "Tuner")
    l_s = cv2.getTrackbarPos("L - S", "Tuner")
    l_v = cv2.getTrackbarPos("L - V", "Tuner")
    u_h = cv2.getTrackbarPos("U - H", "Tuner")
    u_s = cv2.getTrackbarPos("U - S", "Tuner")
    u_v = cv2.getTrackbarPos("U - V", "Tuner")

    lower = np.array([l_h, l_s, l_v])
    upper = np.array([u_h, u_s, u_v])

    mask = cv2.inRange(hsv, lower, upper)
    
    cv2.imshow("Original", frame)
    cv2.imshow("Mask (Make ball white)", mask)

    if cv2.waitKey(1) == ord('q'): break

cap.release()
cv2.destroyAllWindows()
