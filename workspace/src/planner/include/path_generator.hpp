#ifndef PATH_GENERATOR_HPP
#define PATH_GENERATOR_HPP

#include <geometry_msgs/Pose.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/Path.h>

class PathGenerator
{
    public:
        virtual ~PathGenerator() {};

        virtual bool update_path(
            const magellan_messages::MsgZedPose &current_pose,
            const geometry_msgs::Pose &goal_pose,
            const magellan_messages::MsgMagellanOccupancyGrid &world_grid,
            nav_msgs::Path &path) = 0;
};

#endif
