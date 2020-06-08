cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    geometry_msgs
    magellan_messages
)

find_package(OpenCV REQUIRED)

SET(CMAKE_BUILD_TYPE "Release")

catkin_package(
    CATKIN_DEPENDS roscpp image_transport cv_bridge message_generation sensor_msgs magellan_messages geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ros_node.cpp
    src/obstacle_detector.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_debug src/debug_node.cpp
    src/obstacle_detector.cpp
)

set_target_properties(${PROJECT_NAME}_debug PROPERTIES OUTPUT_NAME debug PREFIX "")
target_link_libraries(${PROJECT_NAME}_debug
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
