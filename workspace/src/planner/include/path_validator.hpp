#ifndef PATH_VALIDATOR_HPP
#define PATH_VALIDATOR_HPP

#include <nav_msgs/Path.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <magellan_messages/MsgZedPose.h>

#include "global_map.hpp"

class PathValidator
{
    public:
        virtual bool is_path_valid(
            const magellan_messages::MsgZedPose &current_pose,
            const nav_msgs::Path &path,
            const GlobalMap &global_map) = 0;

        virtual bool is_segment_valid(
            const geometry_msgs::Point &start,
            const geometry_msgs::Point &end,
            const GlobalMap &global_map, 
            bool allow_cone_intersect) = 0;
};

#endif
