import cv2
from picamera2 import Picamera2
import numpy as np
import os

# --- CAMERA CONFIGURATION ---
WIDTH = 640
HEIGHT = 480
FRAMERATE = 60
SHUTTER = 10000
GAIN = 2.0
# -----------------------------

def nothing(x): pass

# --- LOAD PREVIOUS HSV VALUES ---
# Set some safe default values first
l_h_init, l_s_init, l_v_init = 20, 100, 100
u_h_init, u_s_init, u_v_init = 40, 255, 255

if os.path.exists("hsv_values.txt"):
    try:
        with open("hsv_values.txt", "r") as f:
            lines =[line.strip() for line in f if line.strip()]
            if len(lines) >= 2:
                l_h_init, l_s_init, l_v_init =[int(x) for x in lines[0].split(',')]
                u_h_init, u_s_init, u_v_init = [int(x) for x in lines[1].split(',')]
                print("Loaded previous values from hsv_values.txt!")
    except Exception as e:
        print(f"Warning: Could not read hsv_values.txt properly. Using defaults. ({e})")
else:
    print("No hsv_values.txt found. Starting with default values.")

# --- SETUP GUI ---
cv2.namedWindow("Tuner")
# Pass the loaded values as the 3rd parameter to set their starting positions
cv2.createTrackbar("L - H", "Tuner", l_h_init, 179, nothing)
cv2.createTrackbar("L - S", "Tuner", l_s_init, 255, nothing)
cv2.createTrackbar("L - V", "Tuner", l_v_init, 255, nothing)
cv2.createTrackbar("U - H", "Tuner", u_h_init, 179, nothing)
cv2.createTrackbar("U - S", "Tuner", u_s_init, 255, nothing)
cv2.createTrackbar("U - V", "Tuner", u_v_init, 255, nothing)

# --- picamera2 setup ---
print("Initializing camera...")
picam2 = Picamera2()
# EFFICIENT CAPTURE: Request BGR format directly for OpenCV
config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "BGR888"}
)
picam2.configure(config)

picam2.set_controls({
    "FrameDurationLimits": (int(1e6/FRAMERATE), int(1e6/FRAMERATE)),
    "ExposureTime": SHUTTER,
    "AnalogueGain": GAIN,
    "AeEnable": False,
    "AwbEnable": False,
})
picam2.start()
print("Camera started. Tune sliders and press 'q' to quit.")

while True:
    # frame is already in BGR format
    frame = picam2.capture_array()

    # Processing is done on the native BGR frame
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
    
    # --- DISPLAY CORRECTION ---
    # Convert the BGR frame to RGB ONLY for displaying it correctly.
    display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Original (Correct Colors)", display_frame)
    cv2.imshow("Mask (Tune to make ball white)", mask)

    if cv2.waitKey(1) == ord('q'):
        print("\n--- Your Tuned Values ---")
        print(f"LOWER_HSV = np.array([{l_h}, {l_s}, {l_v}])")
        print(f"UPPER_HSV = np.array([{u_h}, {u_s}, {u_v}])")
        print("\n--- Saving Tuned Values ---")
        with open("hsv_values.txt", "w") as f:
            f.write(f"{l_h},{l_s},{l_v}\n")
            f.write(f"{u_h},{u_s},{u_v}\n")
        print("Saved to hsv_values.txt successfully!")
        break

cv2.destroyAllWindows()
picam2.stop()
