#!/usr/bin/env python3
import numpy as np
import os

# Параметры ArUco метки (DICT_4X4_50)
MARKER_SIZE = 5  # 5x5 бит
BORDER_BITS = 1
MARKER_ID = 0
TOTAL_SIZE = 500
BORDER_SIZE = 50

def generate_marker(marker_id, marker_size, border_bits=1):
    # Создаем базовый маркер (псевдокод)
    # В реальности нужно реализовать алгоритм генерации
    # Для примера создадим простой паттерн
    size = marker_size + 2 * border_bits
    marker = np.zeros((size, size), dtype=np.uint8)
    
    # Заполняем границы (черные)
    marker.fill(0)
    
    # Добавляем белый центр
    inner_size = marker_size
    start = border_bits
    end = start + inner_size
    marker[start:end, start:end] = 255
    
    # Добавляем черный квадрат в центре
    center_size = inner_size // 2
    center_start = start + (inner_size - center_size) // 2
    center_end = center_start + center_size
    marker[center_start:center_end, center_start:center_end] = 0
    
    return marker

# Создаем директорию для текстур
os.makedirs("models/aruco_wall/materials/textures", exist_ok=True)

# Генерируем метки
for marker_id in range(10):
    # Генерируем базовое изображение метки
    marker_img = generate_marker(marker_id, MARKER_SIZE, BORDER_BITS)
    
    # Масштабируем до нужного размера
    scaled_img = cv2.resize(marker_img, (TOTAL_SIZE, TOTAL_SIZE), interpolation=cv2.INTER_NEAREST)
    
    # Добавляем белую рамку
    bordered = cv2.copyMakeBorder(
        scaled_img, 
        BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, 
        cv2.BORDER_CONSTANT, 
        value=255
    )
    
    # Сохраняем
    filename = f"models/aruco_wall/materials/textures/aruco_{marker_id}.png"
    cv2.imwrite(filename, bordered)
    print(f"Generated: {filename}")