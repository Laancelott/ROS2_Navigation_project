cmake_minimum_required(VERSION 3.8)
project(nav_pkg)

# Включаем стандарт C++17 (базовый для ROS2 Humble)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Находим обязательные пакеты ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Говорим CMake, что заголовочные файлы (.hpp) нужно искать в папке include
include_directories(include)

# Создаем исполняемый файл нашей ноды из двух исходников
add_executable(navigation_node 
  src/navigation_node.cpp 
  src/d_star_lite.cpp
)

# Линкуем (связываем) библиотеки ROS2 с нашей нодой
ament_target_dependencies(navigation_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
)

# Инсталлируем бинарник, чтобы утилита 'ros2 run' могла его найти
install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Самая важная строчка для ROS2! 
# Она регистрирует наш пакет в системе ament и завершает CMake-скрипт
ament_package()