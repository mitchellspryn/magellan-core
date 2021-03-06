cmake_minimum_required(VERSION 2.8.3)
project(magellan_messages)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    MsgConeDetectorOutput.msg
    MsgSerialPortLine.msg
    MsgMagellanImu.msg
    MsgMagellanDrive.msg
    MsgMagellanOccupancyGrid.msg
    MsgMagellanPlannerDebug.msg
    MsgObstacleDetectorConfig.msg
    MsgVisualOdometryDebug.msg
    MsgVisualOdometry.msg
    MsgLocalObstacleField.msg
    MsgZedPose.msg
    MsgZedSensors.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

