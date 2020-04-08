#ifndef FILTER_HPP
#define FILTER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <magellan_messages/MsgMagellanImu.h>
#include <sensor_msgs/NavSatFix.h>
#include <magellan_messages/MsgMagellanDrive.h>

#include "contracts/global_pose.hpp"

namespace magellan
{
    namespace localization
    {
        class Filter
        {
            public:
                Filter() {}
                virtual ~Filter() {}

                virtual void accept_imu_message(const magellan_messages::MsgMagellanImu::ConstPtr &msg) {};
                virtual void accept_gps_message(const sensor_msgs::NavSatFix::ConstPtr &msg) {};
                virtual void accept_drive_message(const magellan_messages::MsgMagellanDrive::ConstPtr &msg) {};

                virtual void initialize_internal_state() = 0;
                virtual void update_global_pose(ros::Duration dt) = 0;
                virtual const GlobalPose& get_global_pose() const { return this->global_pose; }

                virtual void register_debug_publishers(ros::NodeHandle &nh) {}

            protected:
                GlobalPose global_pose;
        };
    }
}

#endif
