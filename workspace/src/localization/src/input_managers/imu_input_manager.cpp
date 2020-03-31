#include "input_managers/imu_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        ImuInputManager::ImuInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic &measurement_covariance)
        {

        }

        const vector3r_t& ImuInputManager::get_global_acceleration() const
        {
            return this->last_global_acceleration;
        }

        const quaternionr_t& ImuInputManager::get_global_heading() const
        {
            return this->last_global_heading;
        }

        const quaternionr_t& ImuInputManager::get_global_angular_velocity() const
        {
            return this->last_global_angular_velocity;
        }

        const Eigen::Matrix<real_t, state_dimension, state_dimension>& ImuInputManager::get_measurement_covariance() const
        {
            return this->covariance_matrix;
        }

        bool ImuInputManager::data_ready()
        {
            
        }

        void ImuInputManager::add_imu_message(magellan_messages::MsgMagellanImu &msg)
        {

        }

        void ImuInputManager::recompute(const GlobalPose &global_pose)
        {

        }
    }
}
