#include "input_managers/drive_input_manager.hpp"
#include <eigen3/Eigen/src/Core/Map.h>

namespace magellan
{
    namespace localization
    {
        DriveInputManager::DriveInputManager(
                        Eigen::Matrix<real_t, Eigen::Dynamic, 1> &&base_forward_matrix,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&base_forward_covariance_matrix,
                        Eigen::Matrix<real_t, Eigen::Dynamic, 1> &&base_turn_matrix,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&base_turn_covariance_matrix,
                        real_t base_speed,
                        real_t left_power_multiplier)
        {
            pthread_mutex_init(&this->processing_mutex, NULL);
            this->received_commands.reserve(16); //Dummy large value.

            this->base_forward_matrix = std::move(base_forward_matrix);
            this->base_forward_covariance_matrix = std::move(base_forward_covariance_matrix);
            this->base_turn_matrix = std::move(base_turn_matrix);
            this->base_turn_covariance_matrix = std::move(base_turn_covariance_matrix);
            this->base_speed = base_speed;
            this->left_power_multiplier = left_power_multiplier;

            // Set default values
            this->last_control_matrix = Eigen::Matrix<real_t, state_dimension, 1>::Zero();
            this->last_covariance_matrix = Eigen::Matrix<real_t, state_dimension, state_dimension>::Identity() * 0.0001;
        }

        DriveInputManager::~DriveInputManager()
        {
            pthread_mutex_destroy(&this->processing_mutex);
        }

        const Eigen::Matrix<real_t, state_dimension, 1>& DriveInputManager::get_control_matrix() const
        {
            return this->last_control_matrix;
        }

        const Eigen::Matrix<real_t, state_dimension, state_dimension>& DriveInputManager::get_covariance_matrix() const
        {
            return this->last_covariance_matrix;
        }

        void DriveInputManager::add_drive_message(const magellan_messages::MsgMagellanDrive::ConstPtr &msg)
        {
            pthread_mutex_lock(&this->processing_mutex);
            this->received_commands.emplace_back(std::make_pair<real_t, real_t>(msg->left_throttle, msg->right_throttle));
            pthread_mutex_unlock(&this->processing_mutex);
        }

        void DriveInputManager::recompute(const GlobalPose& global_pose)
        {
            pthread_mutex_lock(&this->processing_mutex);

            if (this->received_commands.size() > 0)
            {
                std::pair<real_t, real_t> decomposition = this->decompose_received_commands();

                last_covariance_matrix = (decomposition.first * base_forward_covariance_matrix) + (decomposition.second * base_turn_covariance_matrix);
                last_control_matrix = (decomposition.first * base_forward_matrix) + (decomposition.second * base_turn_matrix);

                this->received_commands.clear();
            }
            
            pthread_mutex_unlock(&this->processing_mutex);
        }

        std::pair<real_t, real_t> DriveInputManager::decompose_received_commands()
        {
            real_t avg_left = 0;
            real_t avg_right = 0;
            real_t num_commands = static_cast<real_t>(this->received_commands.size());

            for (size_t i = 0; i < this->received_commands.size(); i++)
            {
                avg_left += this->received_commands[i].first;
                avg_right += this->received_commands[i].second;
            }

            avg_left /= num_commands;
            avg_right /= num_commands;

            avg_left *= this->left_power_multiplier;

            avg_left /= this->base_speed;
            avg_right /= this->base_speed;

            return std::make_pair<real_t, real_t>(
                    avg_right + avg_left,
                    avg_right - avg_left);
        }
    }
}
