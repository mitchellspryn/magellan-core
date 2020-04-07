#include "input_managers/imu_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        ImuInputManager::ImuInputManager(
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&local_acceleration_covariance,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&global_heading_covariance,
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &&local_angular_velocity_covariance)
        {
            pthread_mutex_init(&this->processing_mutex, NULL);

            this->local_acceleration_tableau = Eigen::Matrix<real_t, 3, 1>::Zero();
            this->global_heading_tableau = Eigen::Matrix<real_t, 3, 1>::Zero();
            this->local_angular_velocity_tableau = Eigen::Matrix<real_t, 3, 1>::Zero();

            this->last_local_acceleration = Eigen::Matrix<real_t, 3, 1>::Zero();
            this->global_heading_tableau = Eigen::Matrix<real_t, 3, 1>::Zero();
            this->local_angular_velocity_tableau = Eigen::Matrix<real_t, 3, 1>::Zero();

            this->local_acceleration_covariance = std::move(local_acceleration_covariance);
            this->global_heading_covariance = std::move(global_heading_covariance);
            this->local_angular_velocity_covariance = std::move(local_angular_velocity_covariance);

            // TODO: Making an optimization assuming covariance matricies are static.
            this->last_local_acceleration_covariance = local_acceleration_covariance;
            this->last_global_heading_covariance = global_heading_covariance;
            this->last_local_angular_velocity_covariance = local_angular_velocity_covariance;
        }

        ImuInputManager::~ImuInputManager()
        {
            pthread_mutex_destroy(&this->processing_mutex);
        }

        const vector3r_t& ImuInputManager::get_local_acceleration() const
        {
            return this->last_local_acceleration;
        }

        const vector3r_t& ImuInputManager::get_global_rotation() const
        {
            return this->last_global_heading;
        }

        const vector3r_t& ImuInputManager::get_local_angular_velocity() const
        {
            return this->last_local_angular_velocity;
        }

        const Eigen::Matrix<real_t, 3, 3>& ImuInputManager::get_local_acceleration_covariance() const
        {
            return this->last_local_acceleration_covariance;
        }

        const Eigen::Matrix<real_t, 3, 3>& ImuInputManager::get_global_heading_covariance() const
        {
            return this->last_global_heading_covariance;
        }

        const Eigen::Matrix<real_t, 3, 3>& ImuInputManager::get_local_angular_velocity_covariance() const
        {
            return this->last_local_angular_velocity_covariance;
        }

        int ImuInputManager::num_unread_messages()
        {
            return this->unread_message_count;
        }

        void ImuInputManager::add_imu_message(const magellan_messages::MsgMagellanImu::ConstPtr &msg)
        {
            pthread_mutex_lock(&this->processing_mutex);
            
            float top_mult = static_cast<float>(this->unread_message_count);
            this->unread_message_count++;
            float bottom_div = static_cast<float>(this->unread_message_count);

            this->local_acceleration_tableau(0, 0) = (((this->local_acceleration_tableau(0, 0) * top_mult) + msg->imu.linear_acceleration.x) / bottom_div);
            this->local_acceleration_tableau(1, 0) = (((this->local_acceleration_tableau(1, 0) * top_mult) + msg->imu.linear_acceleration.y) / bottom_div);
            this->local_acceleration_tableau(2, 0) = (((this->local_acceleration_tableau(2, 0) * top_mult) + msg->imu.linear_acceleration.z) / bottom_div);

            this->local_angular_velocity_tableau(0, 0) = (((this->local_angular_velocity_tableau(0, 0) * top_mult) + msg->imu.angular_velocity.x) / bottom_div);
            this->local_angular_velocity_tableau(1, 0) = (((this->local_angular_velocity_tableau(1, 0) * top_mult) + msg->imu.angular_velocity.y) / bottom_div);
            this->local_angular_velocity_tableau(2, 0) = (((this->local_angular_velocity_tableau(2, 0) * top_mult) + msg->imu.angular_velocity.z) / bottom_div);

            // TODO: this is wrong for heading. It doesn't take calibration of IMU into account. 
            // Need to revisit.
            this->global_heading_tableau(0, 0) = (((this->global_heading_tableau(0, 0) * top_mult) + msg->imu.angular_velocity.x) / bottom_div);
            this->global_heading_tableau(1, 0) = (((this->global_heading_tableau(1, 0) * top_mult) + msg->imu.angular_velocity.y) / bottom_div);
            this->global_heading_tableau(2, 0) = (((this->global_heading_tableau(2, 0) * top_mult) + msg->imu.angular_velocity.z) / bottom_div);
            
            pthread_mutex_unlock(&this->processing_mutex);
        }

        void ImuInputManager::recompute(const GlobalPose &global_pose)
        {
            pthread_mutex_lock(&this->processing_mutex);

            this->last_local_acceleration = local_acceleration_tableau;
            this->last_global_heading = global_heading_tableau;
            this->last_local_angular_velocity = local_angular_velocity_tableau;

            // TODO: are static covariances correct for this sensor?
            // Set in constructor for performance.

            this->unread_message_count = 0;
            pthread_mutex_unlock(&this->processing_mutex);
        }
    }
}
