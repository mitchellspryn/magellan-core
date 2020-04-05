cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_compile_options(-std=c++14) # for make_unique

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    magellan_messages
)

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp image_transport cv_bridge message_generation sensor_msgs magellan_messages
)

include_directories(
    include
    include/localization
    include/localization/filters
    include/localization/sensor_utils
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
    src/node.cpp 
    src/contracts/drive_input_manager_control_point.cpp
    src/contracts/global_pose.cpp
    src/filters/linear_kalman_filter.cpp 
    src/input_managers/drive_input_manager.cpp
    src/input_managers/gps_input_manager.cpp
    src/input_managers/imu_input_manager.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
