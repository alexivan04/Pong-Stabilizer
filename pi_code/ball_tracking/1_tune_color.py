import cv2
import numpy as np
import os

import ball_detector as bd

# Same camera settings, undistortion and plate area as the trackers, so the
# mask you tune here is exactly the mask they use.
#
# Tip: keep L-V low and U-V at 255, and tune H and S instead. Brightness is
# what changes with the room lighting; the ball's saturation barely does.

def nothing(x): pass

# --- LOAD PREVIOUS HSV VALUES ---
(l_h_init, l_s_init, l_v_init), (u_h_init, u_s_init, u_v_init) = bd.load_hsv_range()
if os.path.exists(bd.LEGACY_HSV_FILE):
    print(f"Loaded previous values from {bd.LEGACY_HSV_FILE}!")
else:
    print(f"No {bd.LEGACY_HSV_FILE} found. Starting with default values.")

# --- SETUP GUI ---
cv2.namedWindow("Tuner")
# Pass the loaded values as the 3rd parameter to set their starting positions
cv2.createTrackbar("L - H", "Tuner", int(l_h_init), 179, nothing)
cv2.createTrackbar("L - S", "Tuner", int(l_s_init), 255, nothing)
cv2.createTrackbar("L - V", "Tuner", int(l_v_init), 255, nothing)
cv2.createTrackbar("U - H", "Tuner", int(u_h_init), 179, nothing)
cv2.createTrackbar("U - S", "Tuner", int(u_s_init), 255, nothing)
cv2.createTrackbar("U - V", "Tuner", int(u_v_init), 255, nothing)

map1, map2, new_K = bd.load_undistort()
# plate distance unknown here: use the generous start value (largest circle)
roi = bd.plate_roi(new_K[0, 0], bd.PLATE_Z_START_MM)
roi_mask = bd.plate_roi_mask(new_K[0, 0], bd.PLATE_Z_START_MM)

print("Initializing camera...")
picam2 = bd.start_camera()
print("Camera started. Tune sliders and press 'q' to save and quit.")

while True:
    frame = cv2.remap(picam2.capture_array(), map1, map2, cv2.INTER_LINEAR)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L - H", "Tuner")
    l_s = cv2.getTrackbarPos("L - S", "Tuner")
    l_v = cv2.getTrackbarPos("L - V", "Tuner")
    u_h = cv2.getTrackbarPos("U - H", "Tuner")
    u_s = cv2.getTrackbarPos("U - S", "Tuner")
    u_v = cv2.getTrackbarPos("U - V", "Tuner")

    lower_hsv = np.array([l_h, l_s, l_v])
    upper_hsv = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = cv2.bitwise_and(mask, roi_mask)

    # --- DISPLAY CORRECTION ---
    # Convert the BGR frame to RGB ONLY for displaying it correctly.
    display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.circle(display_frame, (int(roi[0]), int(roi[1])), int(roi[2]), (255, 200, 0), 1)
    cv2.imshow("Original (Correct Colors)", display_frame)
    cv2.imshow("Mask (Tune to make ball white)", mask)

    if cv2.waitKey(1) == ord('q'):
        print("\n--- Your Tuned Values ---")
        print(f"LOWER_HSV = np.array([{l_h}, {l_s}, {l_v}])")
        print(f"UPPER_HSV = np.array([{u_h}, {u_s}, {u_v}])")
        print("\n--- Saving Tuned Values ---")
        with open(bd.LEGACY_HSV_FILE, "w") as f:
            f.write(f"{l_h},{l_s},{l_v}\n")
            f.write(f"{u_h},{u_s},{u_v}\n")
        print(f"Saved to {bd.LEGACY_HSV_FILE} successfully!")
        break

cv2.destroyAllWindows()
picam2.stop()
