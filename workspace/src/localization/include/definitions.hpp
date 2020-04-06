#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace magellan
{
    namespace localization
    {
        constexpr int state_dimension = 15;
        typedef float real_t;
        typedef Eigen::Matrix<real_t, 3, 1> vector3r_t;
        typedef Eigen::Quaternionf quaternionr_t;
    }
}


#endif
