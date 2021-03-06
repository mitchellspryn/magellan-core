#ifndef PATH_GENERATOR_HPP
#define PATH_GENERATOR_HPP

#include <geometry_msgs/Pose.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "global_map.hpp"

class PathGenerator
{
    public:
        virtual ~PathGenerator() {};

        virtual bool update_path(
            const magellan_messages::MsgZedPose& current_pose,
            const geometry_msgs::Pose& goal_pose,
            const GlobalMap& world_grid,
            nav_msgs::Path& path,
            magellan_messages::MsgMagellanOccupancyGrid& debug_grid) = 0;
};

#endif
