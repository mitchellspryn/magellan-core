#ifndef LINEAR_KALMAN_FILTER_HPP
#define LINEAR_KALMAN_FILTER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <magellan_messages/MsgMagellanImu.h>
#include <magellan_messages/MsgMagellanDrive.h>

#include "constants.hpp"
#include "contracts/global_pose.hpp"
#include "contracts/drive_input_control_point.hpp"
#include "filter.hpp"
#include "input_managers/drive_input_manager.hpp"
#include "input_managers/gps_input_manager.hpp"
#include "input_managers/imu_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        class LinearKalmanFilter
        {
            public:
                LinearKalmanFilter(std::string parameter_path);

                void accept_imu_message(magellan_messages::MsgMagellanImu &msg);
                void accept_gps_message(sensor_msgs::NavSatFix &msg);
                void accept_drive_message(magellan_messages::MsgMagellanDrive &msg);

                void update_global_pose(ros::Duration dt);

                void register_debug_publishers(ros::NodeHandle nh);

            private:
                // Runtime variables
                Eigen::Matrix<real_t, state_dimension, 1> state;
                Eigen::Matrix<real_t, state_dimension, state_dimension> state_covariance;

                std::unique_ptr<DriveInputManager> drive_input_manager;
                std::unique_ptr<GpsInputManager> gps_input_manager;
                std::unique_ptr<ImuInputManager> imu_input_manager;

                void initialize_internal_state();

                void initialize_parameters_from_file(std::string parameter_path); 
                Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> read_matrix_from_file(std::fstream stream);
        };
    }
}

#endif
