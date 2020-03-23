cmake_minimum_required(VERSION 2.8.3)
project(drive_from_file)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    magellan_messages
)

catkin_package(
    CATKIN_DEPENDS rospy message_runtime magellan_messages
)

catkin_install_python(PROGRAMS
    scripts/node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

