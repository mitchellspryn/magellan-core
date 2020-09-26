#ifndef PID_MOTOR_SIGNAL_GENERATOR_HPP
#define PID_MOTOR_SIGNAL_GENERATOR_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanDrive.h>

#include "geometry_utils.hpp"
#include "motor_signal_generator.hpp"

#include <ros/ros.h>

class PidMotorSignalGenerator : public MotorSignalGenerator
{
    public:
        virtual magellan_messages::MsgMagellanDrive get_drive_signals(
            const magellan_messages::MsgZedPose &current_pose,
            const nav_msgs::Path &path,
            const geometry_msgs::Point& goal_position) override;

    private:
        const float max_left_wheel_speed = 35.0f;
        const float right_wheel_speed_scale = 55.0f / 40.0f;
        const float turn_sharpness = 2.0f;
        const float distance_start_decay_m = 4.0f;
};

#endif
