#ifndef SIMPLE_PATH_VALIDATOR_HPP
#define SIMPLE_PATH_VALIDATOR_HPP

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <magellan_messages/MsgZedPose.h>

#include "global_map.hpp"
#include "path_validator.hpp"

class SimplePathValidator : public PathValidator
{
    public:
        virtual bool is_path_valid(
            const magellan_messages::MsgZedPose &current_pose,
            const nav_msgs::Path &path,
            const GlobalMap &global_map) override;

        bool is_segment_valid(
            const geometry_msgs::Point &start,
            const geometry_msgs::Point &end,
            const GlobalMap &global_map, 
            bool allow_cone_intersect) override;
};

#endif
