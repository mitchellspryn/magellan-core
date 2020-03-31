#ifndef DRIVE_INPUT_MANAGER_HPP
#define DRIVE_INPUT_MANAGER_HPP

#include <magellan_messages/MsgMagellanDrive.h>
#include <ros/ros.h>
#include <vector>

#include "contracts/drive_input_manager_control_point"

namespace magellan
{
    namespace localization
    {
        class DriveInputManager
        {
            public:
                DriveInputManager(std::vector<DriveInputControlPoint> control_points);

                void add_drive_message(magellan_messages::MsgMagellanDrive &msg);
                void recompute(const GlobalPose const &global_pose);

                const Eigen::Matrix<real_t, state_dimension, 1>& get_control_matrix() const;
                const Eigen::Matrix<real_t, state_dimension, state_dimension> get_covariance_matrix() const;

            private:
                constexpr int control_dimension = 2;

                Eigen::Matrix<real_t, state_dimension, 1> last_control_matrix;
                Eigen::Matrix<real_t, state_dimension, state_dimension> last_covariance_matrix;

                std::vector<DriveInputControlPoint> control_points;
                Eigen::Matrix<real_t, state_dimension, control_dimension> control_matrix_mult;

        }
    }
}

#endif
