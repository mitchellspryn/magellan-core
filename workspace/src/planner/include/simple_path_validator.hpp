#ifndef SIMPLE_PATH_VALIDATOR_HPP
#define SIMPLE_PATH_VALIDATOR_HPP

#include <nav_msgs/Path.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>

#include "path_validator.hpp"

class SimplePathValidator : public PathValidator
{
    public:
        virtual bool is_path_valid(
            const nav_msgs::Path &path,
            const magellan_messages::MsgMagellanOccupancyGrid &global_grid) override;
};

#endif
