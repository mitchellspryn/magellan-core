#ifndef GLOBAL_MAP_HPP
#define GLOBAL_MAP_HPP

#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgObstacleDetection.h>
#include <nav_msgs/OccupancyGrid.h>

class GlobalMap
{
    public:

        void UpdateMap(
                const magellan_messages::MsgZedPose::ConstPtr pose,
                const magellan_messages::MsgObstacleDetection::ConstPtr obstacles);

        

}

#endif
