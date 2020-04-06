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

        const Eigen::Matrix<real_t, 3, 3>GpsInputManager::get_global_position_covariance() const
        {
            return this->last_global_position_covariance;
        }

        int GpsInputManager::num_unread_messages()
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
