import cv2
import numpy as np
import glob

CHECKERBOARD = (8, 6)

objpoints = []
imgpoints = []

objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)

images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)

    if ret:
        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        objpoints.append(objp)
        imgpoints.append(corners.reshape(1, -1, 2))

h, w = gray.shape[:2]

K = np.zeros((3,3))
D = np.zeros((4,1))
rvecs = []
tvecs = []

ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    (w, h),
    K,
    D,
    rvecs,
    tvecs,
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
    cv2.fisheye.CALIB_CHECK_COND |
    cv2.fisheye.CALIB_FIX_SKEW,
    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
)

np.savez("cam_calib_fisheye.npz", K=K, D=D)
print("fisheye calibration saved.")
