# -*- coding: utf-8 -*
#キャリブレーション
import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#images = glob.glob('video*.png')
#gray_images = cv2.cvtColor(images, cv2.COLOR_RGB2GRAY)

gray_images=glob.glob('chess*.png')
#gray_images=[]
#for image in images:
#  gray_images.append(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))

cnt=0

for fname in gray_images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,7),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,7), corners2,ret)
        cv2.imwrite('calib'+str(cnt)+'.png',img)
        cnt+=1

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

#cameraMatrix -> mtx , distCoeffs -> dist
print(mtx, dist)

cv2.destroyAllWindows()