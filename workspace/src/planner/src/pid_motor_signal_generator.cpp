#include "../include/pid_motor_signal_generator.hpp"

magellan_messages::MsgMagellanDrive PidMotorSignalGenerator::get_drive_signals(
    const magellan_messages::MsgZedPose &current_pose,
    const nav_msgs::Path &path,
    const geometry_msgs::Point& goal_position)
{
    magellan_messages::MsgMagellanDrive result;
    if (path.poses.size() == 0)
    {
        result.left_throttle = 0;
        result.right_throttle = 0;
        return result;
    }

    geometry_msgs::Point forward;
    forward.x = 1;
    forward.y = 0;
    forward.z = 0;

    geometry_msgs::Point rotated_to_world = GeometryUtils::RotateByQuaternion(
        forward, 
        current_pose.pose.pose.orientation);

    geometry_msgs::Point loc_to_next_waypoint;
    loc_to_next_waypoint.x = (path.poses[0].pose.position.x - current_pose.pose.pose.position.x);
    loc_to_next_waypoint.y = (path.poses[0].pose.position.y - current_pose.pose.pose.position.y);
    loc_to_next_waypoint.z = 0; //ignore Z difference in distance.

    double distance_to_waypoint = sqrt(
        (loc_to_next_waypoint.x * loc_to_next_waypoint.x)
        +
        (loc_to_next_waypoint.y * loc_to_next_waypoint.y));

    loc_to_next_waypoint.x /= distance_to_waypoint;
    loc_to_next_waypoint.y /= distance_to_waypoint;

    double heading_theta = atan2(rotated_to_world.y, rotated_to_world.x);
    double loc_theta = atan2(loc_to_next_waypoint.y, loc_to_next_waypoint.x);

    double theta_error = heading_theta - loc_theta;

    float dx = goal_position.x - current_pose.pose.pose.position.x;
    float dy = goal_position.y - current_pose.pose.pose.position.y;

    float distance_to_goal = std::sqrt((dx*dx) + (dy*dy));
    float speed_scale_ratio = std::min(1.0f, distance_to_goal / this->distance_start_decay_m);

    double wheel_speed_scale = speed_scale_ratio * this->max_left_wheel_speed;

    if (speed_scale_ratio < 1)
    {
        ROS_ERROR("SLOWING DOWN, ratio is %f", speed_scale_ratio);
    }

    if (theta_error > 0)
    {
        result.left_throttle = this->max_left_wheel_speed * pow(cos(2*theta_error), this->turn_sharpness); 
        result.right_throttle = this->max_left_wheel_speed;
    }
    else
    {
        result.left_throttle = this->max_left_wheel_speed;
        result.right_throttle = this->max_left_wheel_speed * pow(cos(2*theta_error), this->turn_sharpness);
    }

    // Right side runs faster than left side
    result.right_throttle *= this->right_wheel_speed_scale;

    // TODO: we should scale results so that we get slower the closer we get to the final waypoint.

    return result;
}

