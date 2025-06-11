import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_size = 500
border_size = 50

for marker_id in range(4):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    bordered = cv2.copyMakeBorder(img, border_size, border_size, border_size, border_size, 
                                 cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(f"aruco_{marker_id}.png", bordered)
