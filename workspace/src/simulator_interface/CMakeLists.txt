cmake_minimum_required(VERSION 2.8.3)
project(simulator_interface)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    rospy
    std_msgs
    geometry_msgs
    magellan_messages
)

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS rospy image_transport cv_bridge message_runtime sensor_msgs magellan_messages geometry_msgs std_msgs
    DEPENDS OpenCV
)

catkin_install_python(PROGRAMS
    scripts/interface.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
