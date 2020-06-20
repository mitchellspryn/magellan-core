#ifndef PATH_FOLLOWER_HPP
#define PATH_FOLLOWER_HPP

#include <nav_msgs/Path.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanDrive.h>

class PathFollower
{
    public:
        virtual ~PathFollower() {};

        virtual magellan_messages::MsgMagellanDrive get_drive_signals(
            const magellan_messages::MsgZedPose& current_pose,
            const nav_msgs::Path& path) = 0;
};

#endif
