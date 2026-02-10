import cv2
import numpy as np
import glob
import os

# --- CONFIGURATION ---
CHECKERBOARD = (8, 6) # Same as Step 1
IMAGE_SIZE = (640, 480) # Resolution used in Step 1
# ---------------------

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)

images = glob.glob('calibration_images/*.jpg')

print(f"Found {len(images)} images. Processing...")

found = 0
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        objpoints.append(objp)
        
        # Refine corner locations (High Accuracy)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        
        imgpoints.append(corners2)
        found += 1
        print(f"Processed {fname}")

print(f"Used {found} valid images for calibration.")

if found > 0:
    print("Calibrating... (this might take a moment)")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, IMAGE_SIZE, None, None)
    
    print("Calibration Success!")
    print(f"Camera Matrix:\n{mtx}")
    print(f"Distortion Coeffs:\n{dist}")
    
    # Save the result
    np.savez("cam_calib.npz", mtx=mtx, dist=dist)
    print("Saved to 'cam_calib.npz'")
else:
    print("Error: No valid checkerboards found in images.")
