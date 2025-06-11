cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Зависимости (убрать ament_cmake_python)
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Установка SCRIPTS (исполняемые Python-скрипты)
install(PROGRAMS
  scripts/aruco_detector.py
  scripts/aruco_localization.py
  scripts/camera_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Установка ВСЕХ ресурсов за ОДИН раз
install(DIRECTORY 
  config
  description
  launch       # Включаем папку launch
  media       # Включаем ВСЮ папку media (с материалами)
  models
  worlds
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*~" EXCLUDE
)

ament_package()