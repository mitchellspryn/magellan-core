#include "../include/voting_global_map.hpp"

void VotingGlobalMap::update_map(
    const magellan_messages::MsgZedPose &pose,
    const magellan_messages::MsgMagellanOccupancyGrid &obstacles)
{

}

const magellan_messages::MsgMagellanOccupancyGrid& VotingGlobalMap::get_map() const
{
    throw std::runtime_error("Not implemented.");
}

OccupancyGridSquare_t VotingGlobalMap::real_to_grid(const geometry_msgs::Point &real_world_point) const
{
    throw std::runtime_error("Not implemented.");
}

geometry_msgs::Point VotingGlobalMap::grid_to_real(const OccupancyGridSquare_t &square) const 
{
    throw std::runtime_error("Not implemented.");
}
