import cv2

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
img = aruco.generateImageMarker(dictionary, 7, 800)
cv2.imwrite("aruco_5x5_id7.png", img)
