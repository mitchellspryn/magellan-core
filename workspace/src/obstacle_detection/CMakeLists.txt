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
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters features gpu_containers gpu_utils gpu_octree gpu_features)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(Eigen REQUIRED)

SET(CMAKE_BUILD_TYPE "Release")

catkin_package(
    CATKIN_DEPENDS roscpp image_transport cv_bridge message_generation sensor_msgs magellan_messages geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

add_executable(${PROJECT_NAME}_node src/ros_node.cpp
    src/obstacle_detector.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_FEATURES_LIBRARIES}
    ${PCL_GPU_CONTAINERS_LIBRARIES}
    ${PCL_GPU_UTILS_LIBRARIES}
    ${PCL_GPU_OCTREE_LIBRARIES}
    ${PCL_GPU_FEATURES_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
)

add_executable(${PROJECT_NAME}_debug src/debug_node.cpp
    src/obstacle_detector.cpp
)

set_target_properties(${PROJECT_NAME}_debug PROPERTIES OUTPUT_NAME debug PREFIX "")
target_link_libraries(${PROJECT_NAME}_debug
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_FEATURES_LIBRARIES}
    ${PCL_GPU_CONTAINERS_LIBRARIES}
    ${PCL_GPU_UTILS_LIBRARIES}
    ${PCL_GPU_OCTREE_LIBRARIES}
    ${PCL_GPU_FEATURES_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
)
