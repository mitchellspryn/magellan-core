#ifndef LAST_SEEN_GLOBAL_MAP_HPP
#define LAST_SEEN_GLOBAL_MAP_HPP

#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>

#include "global_map.hpp"

class VotingGlobalMap : public GlobalMap
{
    public:
        virtual void update_map(
            const magellan_messages::MsgZedPose &pose,
            const magellan_messages::MsgMagellanOccupancyGrid &obstacles) override;

        virtual const magellan_messages::MsgMagellanOccupancyGrid& get() override;
};

#endif
