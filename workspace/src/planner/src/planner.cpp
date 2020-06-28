#include "../include/planner.hpp"
#include "magellan_messages/MsgMagellanDrive.h"

Planner::Planner()
{
    this->goal_position.x = 0;
    this->goal_position.y = 0;
    this->goal_position.z = 0;

    this->reinitialize();
 
}

magellan_messages::MsgMagellanDrive Planner::run_planner(
    const magellan_messages::MsgZedPose &pose,
    const magellan_messages::MsgMagellanOccupancyGrid &obstacles,
    magellan_messages::MsgMagellanPlannerDebug &debug_msg)
{
    debug_msg.local_obstacle_map = obstacles;
    debug_msg.pose = pose;
    this->global_map->update_map(pose, obstacles);  

    debug_msg.global_obstacle_map = this->global_map->get_map();
    debug_msg.goal = this->get_goal_position();

    if (!this->path_validator->is_path_valid(pose, this->planned_path, *(this->global_map)))
    {
        geometry_msgs::Pose tmp;
        tmp.position = this->goal_position;

        bool path_found = this->path_generator->update_path(pose, tmp, *(this->global_map), this->planned_path);

        if (!path_found)
        {
            ROS_ERROR("Cannot find path to goal.");
            magellan_messages::MsgMagellanDrive result;
            result.left_throttle = 0;
            result.right_throttle = 0;
            debug_msg.path = this->planned_path;
            debug_msg.control_signals = result;
            return result;
        }
    }

    // TODO: remove points as we get close to them.
    // Should the planner do that, should the path validator, or should another module?

    magellan_messages::MsgMagellanDrive signals = this->motor_signal_generator->get_drive_signals(pose, this->planned_path);

    debug_msg.control_signals = signals;
    debug_msg.path = this->planned_path;

    return signals;
}

void Planner::set_goal_position(const geometry_msgs::Point &position)
{
    this->goal_position = position;
    this->reinitialize();
}

const geometry_msgs::Point& Planner::get_goal_position()
{
    return this->goal_position;
}

void Planner::reinitialize()
{
    constexpr float in_to_m = 0.0254f;

    geometry_msgs::Pose tmp;
    tmp.position = this->goal_position;

    magellan_messages::MsgZedPose current_pose;
    current_pose.pose.pose.position.x = 0;
    current_pose.pose.pose.position.y = 0;
    current_pose.pose.pose.position.z = 0;

    this->global_map = std::unique_ptr<LastSeenGlobalMap>(
        new LastSeenGlobalMap(
            tmp,
            5,
            5,
            3 * in_to_m));

    this->motor_signal_generator = std::unique_ptr<MotorSignalGenerator>(
        new PidMotorSignalGenerator());

    this->path_generator = std::unique_ptr<AStarPathGenerator>(
        new AStarPathGenerator(6 * in_to_m));

    this->path_validator = std::unique_ptr<SimplePathValidator>(
        new SimplePathValidator());

    // Must succede, there are no obstacles yet.
    this->path_generator->update_path(
        current_pose, 
        tmp,
        *(this->global_map),
        this->planned_path);
}
