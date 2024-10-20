cmake_minimum_required(VERSION 3.8)
project(gazebo_diff_drive_plugin)

# Gazebo ve ROS2 bağımlılıkları
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

# Kütüphane (plugin) oluşturma
add_library(${PROJECT_NAME} SHARED
  src/diff_drive_gazebo_plugin.cpp
)

# Include dizini
target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
)

# Gerekli kütüphaneleri bağla
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  gazebo_ros
  gazebo
)

# Gazebo bağımlılıkları
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export the package
ament_package()