#ifndef GLOBAL_MAP_HPP
#define GLOBAL_MAP_HPP

#include <geometry_msgs/Point.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>

#include "planner_types.hpp"

class GlobalMap
{
    public:
        virtual ~GlobalMap() {};

        virtual void update_map(
                const magellan_messages::MsgZedPose &pose,
                const magellan_messages::MsgMagellanOccupancyGrid &obstacles) = 0;

        virtual const magellan_messages::MsgMagellanOccupancyGrid& get_map() const = 0;

        virtual OccupancyGridSquare_t real_to_grid(const geometry_msgs::Point &real_world_point) const = 0;
        virtual geometry_msgs::Point grid_to_real(const OccupancyGridSquare_t &square) const = 0;
};

#endif
