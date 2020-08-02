#ifndef GOAL_POINT_ADJUSTER_HPP
#define GOAL_POINT_ADJUSTER_HPP

#include <geometry_msgs/Point.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>

#include "global_map.hpp"

class GoalPointAdjuster
{
    public:
        virtual ~GoalPointAdjuster() {};

        virtual bool adjust_goal_point(
            const GlobalMap& map,
            geometry_msgs::Point& goal_point_estimate) = 0;
};

#endif
