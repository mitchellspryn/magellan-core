#include "input_managers/gps_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        GpsInputManager::GpsInputManager(Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> &measurement_covariance)
        {

        }

        const vector3r_t& GpsInputManager::get_global_position() const
        {
            return this->last_global_position;
        }

        const Eigen::Matrix<real_t, state_dimension, state_dimension>GpsInputManager::get_measurement_covariance() const
        {
            return this->covariance_matrix;
        }

        bool GpsInputManager::data_ready()
        {

        }

        void GpsInputManager::add_gps_message(sensor_msgs::NavSatFix &msg)
        {

        }

        void GpsInputManager::recompute(const GlobalPose &global_pose)
        {

        }
    }
}
