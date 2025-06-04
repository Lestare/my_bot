#!/usr/bin/env python3
import cv2
import numpy as np
import os

# Настройки генерации
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_SIZE = 500  # размер в пикселях
BORDER_SIZE = 50   # размер белой рамки

# Создаем директорию для текстур
os.makedirs("models/aruco_wall/materials/textures", exist_ok=True)

# Генерируем 10 меток с разными ID
for marker_id in range(10):
    # Создаем изображение метки (метод для OpenCV 4.7+)
    marker_image = cv2.aruco.generateImageMarker(DICTIONARY, marker_id, MARKER_SIZE)
    
    # Добавляем белое обрамление
    bordered = cv2.copyMakeBorder(
        marker_image, 
        BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, 
        cv2.BORDER_CONSTANT, 
        value=255
    )
    
    # Сохраняем в формате PNG
    filename = f"models/aruco_wall/materials/textures/aruco_{marker_id}.png"
    cv2.imwrite(filename, bordered)
    print(f"Generated: {filename}")

print("All markers generated!")