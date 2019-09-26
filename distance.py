#!/usr/bin/env python
# -*- coding: utf-8 -*
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

####################################################################

def main():
    cnt=0
    # マーカーサイズ
    marker_length = 0.08 # [m]

    camera_matrix = np.array( [[9.31357583e+03, 0.00000000e+00, 1.61931898e+03],
                          [0.00000000e+00, 9.64867367e+03, 1.92100899e+03],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]] )
    distortion_coeff = np.array( [[ 0.22229833, -6.34741982,  0.01145082,  0.01934784, -8.43093571]   ] )

    try:
        while True:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(client.frame, dictionary, parameters=parameters)
            # 可視化
            aruco.drawDetectedMarkers(client.frame, corners, ids, (0,255,255))

            if len(corners) > 0:
                # マーカーごとに処理
                for i, corner in enumerate(corners):
                    # rvec -> rotation vector, tvec -> translation vector
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distortion_coeff)

                    # < rodoriguesからeuluerへの変換 >

                    # 不要なaxisを除去
                    tvec = np.squeeze(tvec)
                    rvec = np.squeeze(rvec)
                    # 回転ベクトルからrodoriguesへ変換
                    rvec_matrix = cv2.Rodrigues(rvec)
                    rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
                    # 並進ベクトルの転置
                    transpose_tvec = tvec[np.newaxis, :].T
                    # 合成
                    proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                    # オイラー角への変換
                    euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]

                    print("x : " + str(tvec[0]))
                    print("y : " + str(tvec[1]))
                    print("z : " + str(tvec[2]))
                    print("roll : " + str(euler_angle[0]))
                    print("pitch: " + str(euler_angle[1]))
                    print("yaw  : " + str(euler_angle[2]))

                    # 可視化
                    draw_pole_length = marker_length/2 # 現実での長さ[m]
                    aruco.drawAxis(client.frame, camera_matrix, distortion_coeff, rvec, tvec, draw_pole_length)

####################################################################

            cv2.imwrite('distance'+str(cnt)+'.png',client.frame)
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
