#include "../include/simple_path_validator.hpp"
#include "magellan_messages/MsgMagellanOccupancyGrid.h"

bool SimplePathValidator::is_path_valid(
    const nav_msgs::Path &path,
    const magellan_messages::MsgMagellanOccupancyGrid &global_grid)
{
    return true;
}
