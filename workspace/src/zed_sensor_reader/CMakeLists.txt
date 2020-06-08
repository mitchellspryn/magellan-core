cmake_minimum_required(VERSION 2.8.3)
project(zed_sensor_reader)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  magellan_messages
  tf
  visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp image_transport cv_bridge message_generation sensor_msgs magellan_messages visualization_msgs tf
)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

link_directories(
    ${CUDA_LIBRARY_DIRS}
    ${ZED_LIBRARY_DIR}
)

add_executable(${PROJECT_NAME}_node src/zed_sensor_reader.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${CUDA_LIBRARY}
)

add_executable(${PROJECT_NAME}_pcd src/zed_to_pcd.cpp)
set_target_properties(${PROJECT_NAME}_pcd PROPERTIES OUTPUT_NAME pcd PREFIX "")
target_link_libraries(${PROJECT_NAME}_pcd
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${CUDA_LIBRARY}
)
