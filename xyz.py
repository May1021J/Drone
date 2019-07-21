# -*- coding: utf-8 -*
#Webカメラから検出・座標
import cv2
import numpy as np
import sys
#from pyardrone import ARDrone
import logging

aruco = cv2.aruco

# WEBカメラ
cap = cv2.VideoCapture(0)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()
# CORNER_REFINE_NONE, no refinement. CORNER_REFINE_SUBPIX, do subpixel refinement. CORNER_REFINE_CONTOUR use contour-Points
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR

cameraMatrix = np.array( [[1.42068235e+03,0.00000000e+00,9.49208512e+02],
    [0.00000000e+00,1.37416685e+03,5.39622051e+02],
    [0.00000000e+00,0.00000000e+00,1.00000000e+00]] )
distCoeffs = np.array( [1.69926613e-01,-7.40003491e-01,-7.45655262e-03,-1.79442353e-03, 2.46650225e+00] )

cap.set(cv2.CAP_PROP_FPS, 10)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

def main():

    ret, frame = cap.read()
    cnt=0

    # 変換処理ループ
    while ret == True:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary, parameters=parameters)
        #print(corners)
        #print(ids)
        #print(rejectedImgPoints)

        aruco.drawDetectedMarkers(frame, corners, ids, (0,255,0))

        for i, corner in enumerate( corners ):
            points = corner[0].astype(np.int32)
            cv2.polylines(frame, [points], True, (0,255,255))
            cv2.putText(frame, str(ids[i][0]), tuple(points[0]), cv2.FONT_HERSHEY_PLAIN, 1,(0,0,0), 1)

        # rvecs, tvecs, _objPoints =   cv.aruco.estimatePoseSingleMarkers( corners, markerLength, cameraMatrix, distCoeffs[, rvecs[, tvecs[, _objPoints]]] )
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)
        if ids is not None:
            for i in range( ids.size ):
                print( 'rvec {}, tvec {}'.format( rvecs[i], tvecs[i] ))
                print( 'rvecs[{}] {}'.format( i, rvecs[i] ))
                print( 'tvecs[{}] {}'.format( i, tvecs[i] ))
                aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)

        cv2.imwrite('xyz'+str(cnt)+'.png',frame)
        cnt+=1

        # Escキーで終了
        key = cv2.waitKey(50)
        if key == 27: # ESC
            break

        # 次のフレーム読み込み
        ret, frame = cap.read()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass