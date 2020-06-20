#include "../include/pid_path_follower.hpp"

magellan_messages::MsgMagellanDrive PidPathFollower::get_drive_signals(
    const magellan_messages::MsgZedPose &current_pose,
    const nav_msgs::Path &path)
{
    magellan_messages::MsgMagellanDrive result;
    return result;
}
