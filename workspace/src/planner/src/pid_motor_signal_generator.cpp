#include "../include/pid_motor_signal_generator.hpp"

magellan_messages::MsgMagellanDrive PidMotorSignalGenerator::get_drive_signals(
    const magellan_messages::MsgZedPose &current_pose,
    const nav_msgs::Path &path)
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

    double theta_error = acos( 
            (loc_to_next_waypoint.x * rotated_to_world.x)
            +
            (loc_to_next_waypoint.y * rotated_to_world.y));

    if (theta_error > 0)
    {
        result.left_throttle = this->max_left_wheel_speed * cos(2*theta_error); 
        result.right_throttle = this->max_left_wheel_speed;
    }
    else
    {
        result.left_throttle = this->max_left_wheel_speed;
        result.right_throttle = this->max_left_wheel_speed * cos(2*theta_error);
    }

    // Right side runs faster than left side
    result.right_throttle *= this->right_wheel_speed_scale;

    // TODO: we should scale results so that we get slower the closer we get to the final waypoint.

    return result;
}

