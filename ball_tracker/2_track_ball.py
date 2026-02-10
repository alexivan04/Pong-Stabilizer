import cv2
import numpy as np
import time

# --- USER CONFIGURATION ---
# Replace these with the numbers you found in Step 1
LOWER_HSV = np.array([5, 186, 209]) 
UPPER_HSV = np.array([14, 255, 255])

REAL_BALL_DIAMETER_MM = 13.0 # Standard ping pong ball
# --------------------------

# Load Calibration
try:
    with np.load('cam_calib.npz') as data:
        mtx = data['mtx']
        dist = data['dist']
        fx = mtx[0, 0] # Focal length in pixels (x-axis)
        fy = mtx[1, 1] # Focal length in pixels (y-axis)
        cx = mtx[0, 2] # Optical center x
        cy = mtx[1, 2] # Optical center y
except:
    print("Error: Run calibration first!")
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

print("Tracking Started. Press 'q' to quit.")

# Pre-allocate memory for fast processing
kernel = np.ones((3,3), np.uint8) # Small kernel for noise removal

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret: break

    # 1. Convert to HSV (Fastest color space for detection)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Threshold
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # 3. Clean Noise (Erode then Dilate removes white speckles)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # 4. Find Contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find biggest blob (the ball)
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 5: # Filter out tiny noise
            
            # --- OPTIMIZATION: UNDISTORT POINT ONLY ---
            # Instead of undistorting the whole image, we fix just this one coordinate
            # Reshape for opencv function: (N, 1, 2)
            point_distorted = np.array([[[x, y]]], dtype=np.float64)
            
            # This function uses the calibration matrix to fix the fisheye effect on just this point
            point_undistorted = cv2.undistortPoints(point_distorted, mtx, dist, P=mtx)
            
            # Extract corrected coordinates
            ux = point_undistorted[0][0][0]
            uy = point_undistorted[0][0][1]

            # --- MATH: CALCULATE 3D POSITION ---
            # Z = (Focal_Length * Real_Size) / Pixel_Size
            # We use diameter (2*radius) for stability
            pixel_diameter = radius * 2
            
            Z_depth = (fx * REAL_BALL_DIAMETER_MM) / pixel_diameter
            
            # Calculate Real World X and Y based on depth
            # (X, Y) 0,0 is the center of the camera lens
            X_real = ((ux - cx) * Z_depth) / fx
            Y_real = ((uy - cy) * Z_depth) / fy

            # Draw (Use the RAW x,y for drawing on screen, use Real X,Y,Z for logic)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
            
            # Display Real World Coords
            text = f"X:{int(X_real)} Y:{int(Y_real)} Z:{int(Z_depth)} mm"
            cv2.putText(frame, text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # FPS Calculation
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Fast Tracker", frame)
    # cv2.imshow("Mask", mask) # Verify mask if needed, but disable for speed

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
