#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <string>
#include <magellan_messages/MsgMagellanImu.h>
#include <sensor_msgs/NavSatFix.h>

#include "global_pose.hpp"
#include "filter.hpp"

namespace magellan
{
    namespace localization
    {
        class LinearKalmanFilter
        {
            public:
                LinearKalmanFilter(
                        std::string process_covariance_matrix_path,
                        std::string measurement_covariance_matrix_path,
                        std::string control_matrix_path);

                void accept_imu_message(magellan_messages::MsgMagellanImu &msg);
                void accept_gps_message(sensor_msgs::NavSatFix &msg);
                void accept_drive_message(magellan_messages::MsgMagellanDrive &msg);

                void update_global_pose(GlobalPose& pose, ros::Time start_time, ros::Time end_time);

                void register_debug_publishers(ros::NodeHandle nh);

                static constexpr int state_dim = 17;

            private:
                static constexpr int control_dim = 7;

                Eigen::Matrix<float, state_dim, 1> state;
                Eigen::Matrix<float, state_dim, 1> state_estimate;

                Eigen::Matrix<float, state_dim, state_dim> evolution_matrix; // A
                Eigen::Matrix<float, state_dim, control_dim> control_matrix; // B
                Eigen::Matrix<float, state_dim, state_dim> error_covariance; // P

                Eigen::Matrix<float, state_dim, state_dim> process_covariance_matrix; // Q
                Eigen::Matrix<float, state_dim, state_dim> measurement_covariance_matrix; // R
                Eigen::Matrix<float, state_dim, state_dim> kalman_gain; // K
                Eigen::Matrix<float, state_dim, 1> measurement;


        }
    }
}
