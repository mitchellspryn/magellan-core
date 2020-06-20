#ifndef A_STAR_PATH_GENERATOR_HPP
#define A_STAR_PATH_GENERATOR_HPP

#include <geometry_msgs/Pose.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "path_generator.hpp"

class AStarPathGenerator : public PathGenerator
{
    public:
        virtual bool update_path(
            const magellan_messages::MsgZedPose &current_pose,
            const geometry_msgs::Pose &goal_pose,
            const magellan_messages::MsgMagellanOccupancyGrid &world_grid,
            nav_msgs::Path &path) override;
};

#endif
