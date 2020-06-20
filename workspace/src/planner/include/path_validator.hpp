#ifndef PATH_VALIDATOR_HPP
#define PATH_VALIDATOR_HPP

#include <nav_msgs/Path.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>

class PathValidator
{
    public:
        virtual bool is_path_valid(
            const nav_msgs::Path &path,
            const magellan_messages::MsgMagellanOccupancyGrid &global_grid) = 0;
};

#endif
