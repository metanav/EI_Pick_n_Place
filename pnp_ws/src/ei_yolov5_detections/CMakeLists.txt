cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(ei_yolov5_detections VERSION 2.6.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)

set(ei_yolo_v5_blob_name "ei-pnp_yolov5n_320_openvino_2022.1_6shave.blob")
set(ei_yolo_v5_blob "${PROJECT_SOURCE_DIR}/resources/${ei_yolo_v5_blob_name}")

find_package(ament_cmake REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
include_directories( include ${ament_INCLUDE_DIRS})

add_executable(ei_yolov5_spatial_node src/ei_yolov5_spatial_publisher.cpp)

target_link_libraries(ei_yolov5_spatial_node depthai::core)
ament_target_dependencies(ei_yolov5_spatial_node
  camera_info_manager
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs)

add_executable(ei_yolov5_bbox_image_node src/ei_yolov5_bbox_image_publisher.cpp)
target_link_libraries(ei_yolov5_bbox_image_node depthai::core opencv_imgproc opencv_highgui)
ament_target_dependencies(ei_yolov5_bbox_image_node
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs)

target_compile_definitions(ei_yolov5_spatial_node PRIVATE BLOB_NAME="${ei_yolo_v5_blob_name}")

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch FILES_MATCHING PATTERN "*.py")
install(TARGETS ei_yolov5_spatial_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ei_yolov5_bbox_image_node DESTINATION lib/${PROJECT_NAME})

ament_package()

