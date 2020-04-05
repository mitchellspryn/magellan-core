#include "input_managers/drive_input_manager.hpp"

namespace magellan
{
    namespace localization
    {
        DriveInputManager::DriveInputManager(std::vector<DriveInputManagerControlPoint> control_points)
        {

        }

        const Eigen::Matrix<real_t, state_dimension, 1>& DriveInputManager::get_control_matrix() const
        {
            return this->last_control_matrix;
        }

        const Eigen::Matrix<real_t, state_dimension, state_dimension>& DriveInputManager::get_covariance_matrix() const
        {
            return this->last_covariance_matrix;
        }

        bool DriveInputManager::data_ready()
        {

        }

        void DriveInputManager::add_drive_message(magellan_messages::MsgMagellanDrive &msg)
        {

        }

        void DriveInputManager::recompute(const GlobalPose& global_pose)
        {

        }
    }
}
