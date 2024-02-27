cmake_minimum_required(VERSION 3.8)
project(tof_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ENV{ARDUCAM_HEARD_PATH} /usr/include/cpp)

add_executable(tof_interface src/tof_interface.cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

ament_target_dependencies(tof_interface
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

target_include_directories(tof_interface PUBLIC
  $ENV{ARDUCAM_HEARD_PATH}
  ${PCL_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)

target_link_directories(tof_interface PUBLIC
  $ENV{PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(tof_interface
  ArducamDepthCamera
  opencv_highgui
  opencv_videoio
)

install(TARGETS
  tof_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  camera_calibration
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
