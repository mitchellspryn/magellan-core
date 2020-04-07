cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_compile_options(-std=c++14) # for make_unique

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    magellan_messages
    geodesy
)

catkin_package(
    CATKIN_DEPENDS roscpp message_generation sensor_msgs magellan_messages geodesy
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
    src/node.cpp 
    src/contracts/global_pose.cpp
    src/filters/linear_kalman_filter.cpp 
    src/input_managers/drive_input_manager.cpp
    src/input_managers/gps_input_manager.cpp
    src/input_managers/imu_input_manager.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
