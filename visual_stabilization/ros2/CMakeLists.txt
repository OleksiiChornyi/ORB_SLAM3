cmake_minimum_required(VERSION 3.5)

project(drones_stabilization)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Set C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../../
  ${PROJECT_SOURCE_DIR}/../../include
  ${PROJECT_SOURCE_DIR}/../../include/CameraModels
  ${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Specify the executable
add_executable(Stabilization
  src/stabilization.cc
)

# Link dependencies
ament_target_dependencies(Stabilization
  cv_bridge
  rclcpp
  OpenCV
  Eigen3
  Pangolin
  sensor_msgs
)

# Link additional libraries
target_link_libraries(Stabilization
  ${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM3.so
  -lboost_system
)

# Install the executable
install(TARGETS
  Stabilization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
