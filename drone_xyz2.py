# -*- coding: utf-8 -*
#drone_xyz.pyのコピー
#ドローンのカメラから検出・三軸の表示
#
import cv2
import numpy as np
import sys
from pyardrone import ARDrone
import logging

logging.basicConfig(level=logging.DEBUG)

aruco = cv2.aruco #arucoライブラリ
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# ドローンカメラ
client = ARDrone()
client.video_ready.wait()

parameters =  aruco.DetectorParameters_create()
# CORNER_REFINE_NONE, no refinement. CORNER_REFINE_SUBPIX, do subpixel refinement. CORNER_REFINE_CONTOUR use contour-Points
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR

#カメラのキャリブレーション
cameraMatrix = np.array( [[9.31357583e+03 0.00000000e+00 1.61931898e+03],
    [0.00000000e+00 9.64867367e+03 1.92100899e+03],
    [0.00000000e+00 0.00000000e+00 1.00000000e+00]] )
distCoeffs = np.array( [[ 0.22229833 -6.34741982  0.01145082  0.01934784 -8.43093571]] )

#カメラの属性情報
#cap.set(cv2.CAP_PROP_FPS, 10)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

#rvecは回転ベクトル，tvecは並進ベクトル

def main():
    cnt=0
    try:
        while True:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(client.frame, dictionary, parameters=parameters)
            #print(corners)
            #print(ids)
            #print(rejectedImgPoints)

            aruco.drawDetectedMarkers(client.frame, corners, ids, (0,255,0))

            for i, corner in enumerate( corners ):
                points = corner[0].astype(np.int32) #符号あり32ビット整数型
                cv2.polylines(client.frame, [points], True, (0,255,255))    #複数の折れ線の描画
                cv2.putText(client.frame, str(ids[i][0]), tuple(points[0]), cv2.FONT_HERSHEY_PLAIN, 1,(0,0,0), 1)   #番号のテキストの描画

            # rvecs, tvecs, _objPoints =   cv.aruco.estimatePoseSingleMarkers( corners, markerLength, cameraMatrix, distCoeffs[, rvecs[, tvecs[, _objPoints]]] )
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.08, cameraMatrix, distCoeffs) #姿勢推定，markerLength=0.08[m]
            if ids is not None:
                for i in range( ids.size ):
                    #print( 'rvec {}, tvec {}'.format( rvecs[i], tvecs[i] ))
                    #print( 'rvecs[{}] {}'.format( i, rvecs[i] ))
                    #print( 'tvecs[{}] {}'.format( i, tvecs[i] ))
                    aruco.drawAxis(client.frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1) #姿勢推定から座標系軸を描画

            cv2.imwrite('drone_xyz'+str(cnt)+'.png',client.frame)
            cnt+=1

            # Escキーで終了
            key = cv2.waitKey(50)
            if key == 27: # ESC
                break
    finally:
        client.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass