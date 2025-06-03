#!/usr/bin/env python3
import cv2
import numpy as np

# Создаем директорию для текстур
import os
os.makedirs("media/materials/textures", exist_ok=True)

# Генерируем метки
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

for marker_id in range(5):  # Создаем 5 меток
    marker_size = 700  # пикселей
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    
    # Добавляем белую рамку
    bordered = np.ones((marker_size+100, marker_size+100), dtype=np.uint8) * 255
    bordered[50:-50, 50:-50] = marker_img
    
    cv2.imwrite(f"media/materials/textures/tag_6x6_{marker_id}.png", bordered)
    print(f"Generated marker {marker_id}")