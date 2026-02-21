import cv2
import os
import time

# --- CONFIGURATION FOR 9x7 SQUARES ---
# We use (8, 6) because we count internal corners, not squares.
CHECKERBOARD = (8, 6) 
SAVE_FOLDER = "calibration_images"
# -------------------------------------

if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)

def gstreamer_pipeline():
    return (
        "libcamerasrc ! "
        "video/x-raw, width=640, height=480, framerate=60/1, format=NV12 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true sync=false"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

print(f"--- CAPTURE MODE ---")
print(f"Target: Checkerboard with {CHECKERBOARD} INTERNAL CORNERS.")
print("(This corresponds to a board with 9x7 physical squares)")
print("1. Press 's' to save a valid frame.")
print("2. Press 'q' to finish.")

count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Find corners using the updated (8, 6) size
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    display_frame = frame.copy()

    # Draw corners if found
    if ret_corners:
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_corners)
        cv2.putText(display_frame, "BOARD DETECTED (Press 's')", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(display_frame, "Looking for board...", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Capture Calibration", display_frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and ret_corners:
        filename = os.path.join(SAVE_FOLDER, f"img_{count}.jpg")
        cv2.imwrite(filename, frame) 
        print(f"Saved {filename}")
        count += 1
        time.sleep(0.5) 
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
