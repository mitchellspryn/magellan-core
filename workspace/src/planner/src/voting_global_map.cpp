#include "../include/voting_global_map.hpp"

void VotingGlobalMap::update_map(
    const magellan_messages::MsgZedPose &pose,
    const magellan_messages::MsgMagellanOccupancyGrid &obstacles)
{

}

const magellan_messages::MsgMagellanOccupancyGrid& VotingGlobalMap::get()
{
    magellan_messages::MsgMagellanOccupancyGrid g;
    return g;
}
