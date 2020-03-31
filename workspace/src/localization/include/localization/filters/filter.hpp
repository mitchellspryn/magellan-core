#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <magellan_messages/MsgMagellanImu.h>
#include <sensor_msgs/NavSatFix.h>
#include <magellan_messages/MsgMagellanDrive.h>

#include "global_pose.hpp"

namespace magellan
{
    namespace localization
    {
        class Filter
        {
            public:
                virtual ~Filter() = {};

                virtual void accept_imu_message(magellan_messages::MsgMagellanImu &msg) = {};
                virtual void accept_gps_message(sensor_msgs::NavSatFix &msg) = {};
                virtual void accept_drive_message(magellan_messages::MsgMagellanDrive &msg) = {};

                virtual void is_initialized() = 0;
                virtual void update_global_pose(GlobalPose& pose, ros::Time start_time, ros::Time end_time) = 0;

                virtual void register_debug_publishers(ros::NodeHandle &nh);
        }
    }
}

