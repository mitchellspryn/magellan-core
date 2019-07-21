cmake_minimum_required(VERSION 2.8.3)
project(magellan_drive_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    magellan_messages
    message_runtime
)
 
catkin_package(
    CATKIN_DEPENDS roscpp message_generation magellan_serial_port magellan_messages
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/magellan_drive_hardware_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
