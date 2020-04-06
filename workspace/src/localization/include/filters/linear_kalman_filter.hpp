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

#include "contracts/global_pose.hpp"
#include "contracts/drive_input_manager_control_point.hpp"
#include "definitions.hpp"
#include "filter.hpp"
#include "input_managers/drive_input_manager.hpp"
#include "input_managers/gps_input_manager.hpp"
#include "input_managers/imu_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        class LinearKalmanFilter : public Filter
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

                Eigen::Matrix<real_t, 3, 1> rotate_vector(
                        const Eigen::Matrix<real_t, 3, 3> &rotation_matrix,
                        const Eigen::Matrix<real_t, 3, 3> &original_vector);

                Eigen::Matrix<real_t, 3, 3> rotate_covariance_matrix(
                        const Eigen::Matrix<real_t, 3, 3> &rotation_matrix, 
                        const Eigen::Matrix<real_t, 3, 3> &original_covariance);

                Eigen::Matrix<real_t, 3, 3> roll_pitch_yaw_to_rotation_matrix(const vector3r_t &rpy_vec);

                void initialize_parameters_from_file(std::string parameter_path); 
                DriveInputManagerControlPoint read_drive_control_point_from_file(std::ifstream &stream, std::string expected_matrix_name);
                Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> read_matrix_from_file(std::ifstream &stream, std::string expected_matrix_name);
        };
    }
}

#endif
