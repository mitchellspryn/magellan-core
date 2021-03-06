#ifndef MOTOR_SIGNAL_GENERATOR_HPP
#define MOTOR_SIGNAL_GENERATOR_HPP

#include <nav_msgs/Path.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanDrive.h>
#include <geometry_msgs/Point.h>

class MotorSignalGenerator
{
    public:
        virtual ~MotorSignalGenerator() {};

        virtual magellan_messages::MsgMagellanDrive get_drive_signals(
            const magellan_messages::MsgZedPose& current_pose,
            const nav_msgs::Path& path,
            const geometry_msgs::Point& goal_position) = 0;
};

#endif
