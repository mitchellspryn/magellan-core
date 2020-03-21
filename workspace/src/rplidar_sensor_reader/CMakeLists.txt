cmake_minimum_required(VERSION 2.8.3)
project(rplidar_sensor_reader)

add_compile_options(-std=c++11)

find_library(RPLIDAR_SDK_LIB_LOCATION NAMES librplidar_sdk.a PATHS /usr/lib/librplidar_sdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  magellan_messages
  message_runtime
)

catkin_package(
    CATKIN_DEPENDS roscpp message_generation magellan_serial_port magellan_messages sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/rplidar_sensor_reader.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${RPLIDAR_SDK_LIB_LOCATION}
  pthread
)
