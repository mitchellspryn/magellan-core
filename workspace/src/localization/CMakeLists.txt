cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_compile_options(-std=c++14) # for make_unique

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    sensor_msgs
    magellan_messages
    geodesy
)

catkin_package(
    CATKIN_DEPENDS roscpp rosbag message_generation sensor_msgs magellan_messages geodesy
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

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_run_filter 
    src/run_filter.cpp 
    src/contracts/global_pose.cpp
    src/filters/linear_kalman_filter.cpp 
    src/input_managers/drive_input_manager.cpp
    src/input_managers/gps_input_manager.cpp
    src/input_managers/imu_input_manager.cpp)

add_dependencies(${PROJECT_NAME}_run_filter
    ${catkin_EXPORTED_TARGETS})

set_target_properties(${PROJECT_NAME}_run_filter PROPERTIES OUTPUT_NAME run_filter PREFIX "")
target_link_libraries(${PROJECT_NAME}_run_filter
    ${catkin_LIBRARIES}
)

