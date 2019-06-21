#!/usr/bin/env python
# -*- coding: utf-8 -*
import cv2
aruco = cv2.aruco
dir(aruco)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker = aruco.drawMarker(dictionary, 0, 64)
cv2.imshow('0.64', marker)
cv2.imwrite('0.64.png', marker)

marker = aruco.drawMarker(dictionary, 1, 64)
cv2.imshow('1.64', marker)
cv2.imwrite('1.64.png', marker)

marker = aruco.drawMarker(dictionary, 2, 64)
cv2.imshow('2.64', marker)
cv2.imwrite('2.64.png', marker)

marker = aruco.drawMarker(dictionary, 3, 64)
cv2.imshow('3.64', marker)
cv2.imwrite('3.64.png', marker)

marker = aruco.drawMarker(dictionary, 4, 64)
cv2.imshow('4.64', marker)
cv2.imwrite('4.64.png', marker)

cv2.waitKey(0)
cv2.destroyAllWindows()
f:id:pongsuke:20170609123603p:plain f:id:pongsuke:20170609123609p:plain f:id:pongsuke:20170609123612p:plain f:id:pongsuke:20170609123616p:plain f:id:pongsuke:20170609123619p:plain

マーカーの検出
検出後に、印を書き込んでいます。

cv2.drawDetectedMarkers を使えば、３シュルのデータを一発で書き込んでくれますが、どんなデータが帰ってきているのかを把握するために、１つずつ書いてみました。

#!/usr/bin/env python
# -*- coding: utf-8 -*
import cv2
import numpy as np

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

#img = cv2.imread('100.jpg')
#img = cv2.imread('101.jpg')
img = cv2.imread('DSC_3067.jpg')
img = cv2.resize(img, None, fx=0.5, fy=0.5)
cv2.imwrite('resize.png', img)

corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary)
print corners
print ids
#print rejectedImgPoints

#aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))

for i, corner in enumerate( corners ):
    points = corner[0].astype(np.int32)
    cv2.polylines(img, [points], True, (0,255,255))
    print type(points[0])
    cv2.putText(img, str(ids[i][0]), tuple(points[0]), cv2.FONT_HERSHEY_PLAIN, 1,(0,0,0), 1)

cv2.imshow('drawDetectedMarkers', img)
cv2.imwrite('drawDetectedMarkers.png', img)


cv2.waitKey(0)
cv2.destroyAllWindows()
