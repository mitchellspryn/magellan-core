cmake_minimum_required(VERSION 2.8.3)
project(xtion_reader)

add_compile_options(-std=c++11)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(OpenNI2 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp image_transport cv_bridge message_generation sensor_msgs 
    DEPENDS OpenCV OpenNI2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENNI2_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/xtion_reader.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENNI2_LIBRARIES}
)
