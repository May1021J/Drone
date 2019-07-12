# -*- coding: utf-8 -*-
import numpy as np
import cv2 
import sys
 
aruco = cv2.aruco #arucoライブラリ
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

def arReader(): 
    print(cv2.getBuildInformation()) #詳細の確認
    cap = cv2.VideoCapture(0) #ビデオキャプチャの開始
    cnt=0

    while True:
 
        ret, frame = cap.read() #ビデオキャプチャから画像を取得
        if frame is None: break
 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary) #マーカを検出
        print(corners,ids)
        aruco.drawDetectedMarkers(frame, corners, ids, (0,255,0)) #検出したマーカに描画する

        cv2.imwrite('result'+cnt+'.png',frame)
        cnt+=1

    cap.release() #ビデオキャプチャのメモリ解放
    cv2.destroyAllWindows() #すべてのウィンドウを閉じる

if __name__ == '__main__':
    arReader()