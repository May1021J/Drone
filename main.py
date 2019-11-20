#!/usr/bin/env python
# -*- coding: utf-8 -*
#検出・計算・動作の一連の流れ

import cv2
import numpy as np
import sys
from pyardrone import ARDrone
import logging
from socket import socket, AF_INET, SOCK_DGRAM

HOST = ''   
PORT = 5000

# ソケットを用意
s = socket(AF_INET, SOCK_DGRAM)
# バインドしておく
s.bind((HOST, PORT))

logging.basicConfig(level=logging.DEBUG)

aruco = cv2.aruco #arucoライブラリ
# ドローンカメラ
client = ARDrone()
client.video_ready.wait()

#client.emergency
#if client.state.emergency_mask is True:
#    client.emergency()

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()
# CORNER_REFINE_NONE, no refinement. CORNER_REFINE_SUBPIX, do subpixel refinement. CORNER_REFINE_CONTOUR use contour-Points
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR

####################################################################

#ドローンの動作部分
def drone_chage_state(forward,land,hover):
    print('drone change state',forward,land,hover)
    if land:
        client.land()
    elif hover:
        client.hover()
    else:
        client.move(forward=forward)


#処理の分岐 0.5m=8 1m=16 1.5m=24 2m=32
def calc(tvec, euler):
    print("z = " + str(tvec[2]))
    forward, land, hover = 0, False, False
    if not tvec.any():
        return

    # 1.5m以上離れたら着陸する
    if tvec[2] >= 24:
        print("1 --> 着陸")
        land=True

    # 0.5m < tvec < 1.5m のとき速度0.5で飛行する
    elif 8 < tvec[2] and tvec[2] < 24:
        print("2 --> 速度0.3で直進")
        forward=0.3

    # どれにもあてはまらなかったらホバリング
    else:
        print("3 --> ホバリング")
        hover=True

    drone_chage_state(forward,land,hover)

#検出・計算・追尾
def tracking():
    # マーカーサイズ
    marker_length = 0.1 # [m]

    camera_matrix = np.array( [[9.31357583e+03, 0.00000000e+00, 1.61931898e+03],
                              [0.00000000e+00, 9.64867367e+03, 1.92100899e+03],
                              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]] )
    distortion_coeff = np.array( [[ 0.22229833, -6.34741982,  0.01145082,  0.01934784, -8.43093571]] )

    try:

        forward, land, hover = 0, False, False

        while True:
            print("############################")
            print('states',client.state.emergency_mask)
#            print("---detectMarkersの前---")
            corners, ids, rejectedImgPoints = aruco.detectMarkers(client.frame, dictionary, parameters=parameters)
#            print("---detectMarkersの後---")
            # 可視化
            aruco.drawDetectedMarkers(client.frame, corners, ids, (0,255,255))
#            print("---if文前---")

            if len(corners) > 0:
                # マーカーごとに処理
                for i, corner in enumerate(corners):
                    # rvec -> rotation vector, tvec -> translation vector
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distortion_coeff)

                    # <<< rodoriguesからeuluerへの変換 >>>

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

#                    print("x : " + str(tvec[0]))
#                    print("y : " + str(tvec[1]))
#                    print("z : " + str(tvec[2]))
#                    print("roll : " + str(euler_angle[0]))
#                    print("pitch: " + str(euler_angle[1]))
#                    print("yaw  : " + str(euler_angle[2]))

                    # 可視化
                    draw_pole_length = marker_length/2 # 現実での長さ[m]
                    aruco.drawAxis(client.frame, camera_matrix, distortion_coeff, rvec, tvec, draw_pole_length)

                    calc(tvec, euler_angle)

            # マーカが検出できなかったらホバリング
            else:
                print("4 --> 未検出・ホバリング")
                hover=True
                drone_chage_state(forward,land,hover)
    except:
        pass

def main():
    try:
        while True:
            # 受信
            msg, address = s.recvfrom(8192)
            print(f"message: {msg}\nfrom: {address}")

            if msg == "takeoff":    #離陸
                client.takeoff()
            elif msg == "tracking": #追尾
                tracking()
            elif msg == "land":     #着陸
                client.land()
            else:
                pass
    finally:
        s.close()
        client.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Ctrl+Cで停止しました")
    finally:
        client.land()