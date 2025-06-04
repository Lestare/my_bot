import cv2
import numpy as np
import os

output_dir = "models/aruco_wall/materials/textures"
os.makedirs(output_dir, exist_ok=True)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

for marker_id in range(10):
    marker_size = 500
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size, marker_image, 1)
    
    # Добавляем белое обрамление
    bordered = cv2.copyMakeBorder(marker_image, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
    
    cv2.imwrite(f"{output_dir}/aruco_{marker_id}.png", bordered)
    print(f"Generated marker {marker_id}")