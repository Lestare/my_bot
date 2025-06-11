cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Находим зависимости
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Установка Python скриптов
install(PROGRAMS
  scripts/aruco_detector.py
  scripts/aruco_localization.py
  scripts/camera_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Установка ресурсов с сохранением структуры
install(DIRECTORY 
  config/
  description/
  media/
  models/
  worlds/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*~" EXCLUDE
)

# Установка материалов отдельно
install(DIRECTORY
  media/materials/
  DESTINATION share/${PROJECT_NAME}/media
)

# Установка launch-файлов
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py"  # Разрешаем установку .py файлов
)

# Установка остальных ресурсов
install(DIRECTORY 
  config 
  description 
  worlds 
  media 
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()