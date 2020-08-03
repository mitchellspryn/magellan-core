#ifndef SIMPLE_GOAL_POINT_ADJUSTER_HPP
#define SIMPLE_GOAL_POINT_ADJUSTER_HPP

#include <memory>
#include <queue>
#include <unordered_set>
#include "global_map.hpp"
#include "magellan_messages/MsgMagellanOccupancyGrid.h"
#include "goal_point_adjuster.hpp"

class SimpleGoalPointAdjuster : public GoalPointAdjuster
{
    public:
        virtual bool adjust_goal_point(
            const GlobalMap& map,
            geometry_msgs::Point& goal_point_estimate) override;

    private:
        static constexpr float max_adjust_distance_m = 1.0f;

        geometry_msgs::Point find_centroid(
            const GlobalMap& grid,
            int startW,
            int startH);
};

#endif
