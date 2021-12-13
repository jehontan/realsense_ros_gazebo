cmake_minimum_required(VERSION 3.5)
project(realsense_ros2_gazebo)

# ROS1 Reference
# find_package(catkin REQUIRED COMPONENTS
#   gazebo_dev
#   gazebo_msgs
#   rclcpp
#   nodelet
#   sensor_msgs
#   urdf
#   tf
#   tf2_ros
#   rosconsole
#   diagnostic_updater
#   std_msgs
#   gazebo_ros
#   image_transport
#   camera_info_manager
# )

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# find_package(gazebo REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_fastrtps_cpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(realsense_gazebo_plugin src/RealSensePlugin.cpp src/gazebo_ros_realsense.cpp)
# target_link_libraries(realsense_gazebo_plugin ${rclcpp_LIBRARIES} ${GAZEBO_LIBRARIES})

ament_target_dependencies(realsense_gazebo_plugin
  rclcpp
  std_msgs
  sensor_msgs
  gazebo_dev
)

ament_export_dependencies(
  gazebo_msgs
  rclcpp
  geometry_msgs
  sensor_msgs
  urdf
  tf2_ros
  std_msgs
  gazebo_ros
  image_transport
  camera_info_manager
)

ament_export_include_directories(include)

ament_export_libraries(realsense_gazebo_plugin)

ament_package()

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include
)

install (TARGETS realsense_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY meshes
    DESTINATION share
)

install(DIRECTORY xacro
    DESTINATION share
)