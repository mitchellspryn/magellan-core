#include "filters/linear_kalman_filter.hpp"

namespace magellan
{
    namespace localization
    {
        LinearKalmanFilter::LinearKalmanFilter(std::string parameter_path)
        {

        }

        void LinearKalmanFilter::accept_imu_message(magellan_messages::MsgMagellanImu &msg)
        {

        }

        void LinearKalmanFilter::accept_gps_message(sensor_msgs::NavSatfix &msg)
        {

        }

        void LinearKalmanFilter::accept_drive_message(magellan_messages::MsgMagellanDrive &msg)
        {

        }

        void LinearKalmanFilter::update_global_pose(GlobalPose& pose, ros::Time start_time, ros::Time end_time)
        {

        }

        void LinearKalmanfilter::register_debug_publishers(ros::NodeHandle nh)
        {

        }
    }
}
