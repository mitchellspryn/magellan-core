cmake_minimum_required(VERSION 2.8.3)
project(status_webpage)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    rospy
    std_msgs
    geometry_msgs
    magellan_messages
)

find_package(OpenCV REQUIRED)
find_package(OpenNI2 REQUIRED)

catkin_package(
    CATKIN_DEPENDS rospy image_transport cv_bridge message_runtime sensor_msgs magellan_messages geometry_msgs std_msgs 
)

install(PROGRAMS
    scripts/webpage.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
