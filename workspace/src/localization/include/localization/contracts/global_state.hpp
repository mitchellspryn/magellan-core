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
                vector3r_t GlobalPosition;
                quaternionr_t GlobalRotation;

                Eigen::Matrix<real_t, 3, 3> GlobalPositionCovariance;
                Eigen::Matrix<real_t, 4, 4> GlobalRotationCovariance;

                GlobalPose()
        };
    }
}

#endif
