#ifndef GLOBAL_MAP_HPP
#define GLOBAL_MAP_HPP

#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>

class GlobalMap
{
    public:
        void update_map(
                const magellan_messages::MsgZedPose &pose,
                const magellan_messages::MsgMagellanOccupancyGrid &obstacles);

        const magellan_messages::MsgMagellanOccupancyGrid& get();
};

#endif
