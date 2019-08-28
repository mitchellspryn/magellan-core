cmake_minimum_required(VERSION 2.8.3)
project(magellan_messages)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    cone_detector/MsgConeDetectorOutput.msg
    magellan_serial_port/MsgSerialPortLine.msg
    magellan_imu/MsgMagellanImu.msg
    magellan_drive/MsgMagellanDrive.msg
    visual_odometry/MsgVisualOdometryDebug.msg
    visual_odometry/MsgVisualOdometry.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

