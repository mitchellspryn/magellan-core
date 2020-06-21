#ifndef LAST_SEEN_GLOBAL_MAP_HPP
#define LAST_SEEN_GLOBAL_MAP_HPP

#include <stdexcept>

#include <geometry_msgs/Point.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>

#include "global_map.hpp"
#include "planner_types.hpp"

class VotingGlobalMap : public GlobalMap
{
    public:
        virtual void update_map(
            const magellan_messages::MsgZedPose &pose,
            const magellan_messages::MsgMagellanOccupancyGrid &obstacles) override;

        virtual const magellan_messages::MsgMagellanOccupancyGrid& get_map() const override;

        virtual OccupancyGridSquare_t real_to_grid(const geometry_msgs::Point &real_world_point) const override;

        virtual geometry_msgs::Point grid_to_real(const OccupancyGridSquare_t &square) const override;
};

#endif
