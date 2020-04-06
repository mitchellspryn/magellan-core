#ifndef GLOBAL_STATE_HPP
#define GLOBAL_STATE_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "definitions.hpp"


namespace magellan
{
    namespace localization
    {
        class GlobalPose
        {
            public:
                vector3r_t global_position;
                vector3r_t global_rotation;
                vector3r_t global_acceleration;
                vector3r_t global_velocity;
                vector3r_t global_angular_velocity;

                Eigen::Matrix<real_t, 3, 3> global_position_covariance;
                Eigen::Matrix<real_t, 3, 3> global_rotation_covariance;
                Eigen::Matrix<real_t, 3, 3> global_acceleration_covariance;
                Eigen::Matrix<real_t, 3, 3> global_velocity_covariance;
                Eigen::Matrix<real_t, 3, 3> global_angular_velocity_covariance;

                GlobalPose();
        };
    }
}

#endif
