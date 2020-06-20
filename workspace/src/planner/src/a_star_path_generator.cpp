#include "../include/a_star_path_generator.hpp"
#include "magellan_messages/MsgMagellanOccupancyGrid.h"
#include "magellan_messages/MsgZedPose.h"

bool AStarPathGenerator::update_path(
    const magellan_messages::MsgZedPose &current_pose,
    const geometry_msgs::Pose &goal_pose,
    const magellan_messages::MsgMagellanOccupancyGrid &world_grid,
    nav_msgs::Path &path) 
{
    return true;
}
