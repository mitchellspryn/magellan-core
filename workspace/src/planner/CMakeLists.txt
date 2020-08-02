cmake_minimum_required(VERSION 2.8.3)
project(planner)

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

SET(CMAKE_BUILD_TYPE "Debug")

catkin_package(
    CATKIN_DEPENDS roscpp image_transport cv_bridge message_generation sensor_msgs magellan_messages geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ros_node.cpp
    src/a_star_path_generator.cpp
    src/geometry_utils.cpp
    src/last_seen_global_map.cpp
    src/pid_motor_signal_generator.cpp
    src/planner.cpp
    src/simple_goal_point_adjuster.cpp
    src/simple_path_validator.cpp
    src/voting_global_map.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

