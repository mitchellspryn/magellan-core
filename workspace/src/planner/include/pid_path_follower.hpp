#ifndef PID_PATH_FOLLOWER_HPP
#define PID_PATH_FOLLOWER_HPP

#include <nav_msgs/Path.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanDrive.h>

#include "path_follower.hpp"

class PidPathFollower : PathFollower
{
    public:
        virtual magellan_messages::MsgMagellanDrive get_drive_signals(
            const magellan_messages::MsgZedPose &current_pose,
            const nav_msgs::Path &path) override;
};

#endif
