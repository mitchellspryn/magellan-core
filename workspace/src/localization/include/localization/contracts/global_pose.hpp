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
                quaternionr_t global_rotation;

                Eigen::Matrix<real_t, 3, 3> global_position_covariance;
                Eigen::Matrix<real_t, 4, 4> global_rotation_covariance;

                GlobalPose();
        };
    }
}

#endif
