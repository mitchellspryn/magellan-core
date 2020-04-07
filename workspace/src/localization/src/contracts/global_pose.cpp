#include "contracts/global_pose.hpp"
#include <eigen3/Eigen/src/Core/Map.h>
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace magellan
{
    namespace localization
    {
        GlobalPose::GlobalPose()
        {
            this->global_position = vector3r_t::Zero();
            this->global_rotation = vector3r_t::Zero();
            this->global_acceleration = vector3r_t::Zero();
            this->global_velocity = vector3r_t::Zero();
            this->global_angular_velocity = vector3r_t::Zero();

            this->global_position_covariance = Eigen::Matrix<real_t, 3, 3>::Zero();
            this->global_rotation_covariance = Eigen::Matrix<real_t, 3, 3>::Zero();
            this->global_acceleration_covariance = Eigen::Matrix<real_t, 3, 3>::Zero();
            this->global_velocity_covariance = Eigen::Matrix<real_t, 3, 3>::Zero();
            this->global_angular_velocity_covariance = Eigen::Matrix<real_t, 3, 3>::Zero();
        }
    }
}

