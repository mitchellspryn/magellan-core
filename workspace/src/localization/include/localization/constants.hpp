#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace magellan
{
    namespace localization
    {
        constexpr int state_dimension = 17;
        typedef float real_t;
        typedef Eigen::Matrix<float, 3, 1> vector3r_t;
        typedef Eigen::Quaternionf quaternionr_t;
    }
}


#endif
