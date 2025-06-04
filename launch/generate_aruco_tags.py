import cv2
import cv2.aruco as aruco
import os

# Создаем директорию для текстур
os.makedirs("models/aruco_wall/materials/textures", exist_ok=True)

# Настройки генерации
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_size = 500  # размер в пикселях

# Генерируем 10 меток с разными ID
for marker_id in range(10):
    # Создаем изображение метки
    img = aruco.generateImageMarker(dictionary, marker_id, marker_size)
    
    # Сохраняем в формате PNG
    filename = f"models/aruco_wall/materials/textures/aruco_{marker_id}.png"
    cv2.imwrite(filename, img)
    print(f"Generated: {filename}")

print("All markers generated!")