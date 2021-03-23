import numpy as np
import cv2
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.00001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cap = cv2.VideoCapture(0)

image_counter = 0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if np.shape(frame) == ():
        continue
    # Our operations on the frame come here
    flip = cv2.flip(frame, -1)
    gray = cv2.cvtColor(flip, cv2.COLOR_BGR2GRAY)


    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (5,7), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (3,3), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(flip, (7,5), corners2, ret)
        cv2.imshow('chessboard found', flip)
        if cv2.waitKey(500) & 0xFF == ord('q'):
            break
    else:
        # Display the resulting frame
        cv2.imshow('chessboard not found', flip)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

ret, frame = cap.read()
h,  w = frame.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if np.shape(frame) == ():
        continue
    # undistort
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    cv2.imshow('original', frame)
    cv2.imshow('undistorted', dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
