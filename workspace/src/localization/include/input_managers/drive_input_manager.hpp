#ifndef DRIVE_INPUT_MANAGER_HPP
#define DRIVE_INPUT_MANAGER_HPP

#include <magellan_messages/MsgMagellanDrive.h>
#include <pthread.h>
#include <ros/ros.h>
#include <vector>

#include "contracts/global_pose.hpp"

#include "definitions.hpp"

namespace magellan
{
    namespace localization
    {
        class DriveInputManager
        {
            public:
                DriveInputManager(
                        Eigen::Matrix<real_t, Eigen::Dynamic, 1> &&base_forward_matrix, 
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&base_forward_covariance_matrix,
                        Eigen::Matrix<real_t, Eigen::Dynamic, 1> &&base_turn_matrix,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&base_turn_covariance_matrix,
                        real_t base_speed,
                        real_t left_power_multiplier);
                ~DriveInputManager();

                const Eigen::Matrix<real_t, state_dimension, 1>& get_control_matrix() const;
                const Eigen::Matrix<real_t, state_dimension, state_dimension>& get_covariance_matrix() const;
                void add_drive_message(const magellan_messages::MsgMagellanDrive::ConstPtr &msg);
                void recompute(const GlobalPose &global_pose);

            private:
                static constexpr int control_dimension = 2;

                pthread_mutex_t processing_mutex;

                Eigen::Matrix<real_t, state_dimension, 1> last_control_matrix;
                Eigen::Matrix<real_t, state_dimension, state_dimension> last_covariance_matrix;

                // For the bot, the left motors are more powerful than the right motors.
                // This factor represents this disparity.
                // It is computed such that
                // 
                // left_power_multiplier * left_speed = right_speed
                // 
                // for driving straight.
                real_t left_power_multiplier = 0;
                real_t base_speed = 0;

                Eigen::Matrix<real_t, state_dimension, 1> base_forward_matrix;
                Eigen::Matrix<real_t, state_dimension, state_dimension> base_forward_covariance_matrix;
                Eigen::Matrix<real_t, state_dimension, 1> base_turn_matrix;
                Eigen::Matrix<real_t, state_dimension, state_dimension> base_turn_covariance_matrix;
                std::vector<std::pair<real_t, real_t>> received_commands;

                // Decompose into straight, turn eigenvalues
                std::pair<real_t, real_t> decompose_received_commands();

        };
    }
}

#endif
