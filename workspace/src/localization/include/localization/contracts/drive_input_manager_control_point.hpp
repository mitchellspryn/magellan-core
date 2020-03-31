#ifndef DRIVE_INPUT_MANAGER_CONTROL_POINT_HPP
#define DRIVE_INPUT_MANAGER_CONTROL_POINT_HPP

#include <eigen3/Eigen/Dense>
#include <utility>

#include "definitions.hpp"

namespace magellan
{
    namespace localization
    {
        class DriveInputManagerControlPoint
        {
            public:
                DriveInputManagerControlPoint(std::pair<real_t, real_t> drive_params, 
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> control_matrix, 
                        Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> covariance_matrix);

                const std::pair<real_t, real_t>& get_drive_params() const;
                const Eigen::Matrix<real_t, state_dimension, 1>& get_control_matrix() const;
                const Eigen::Matrix<real_t, state_dimension, state_dimension>& get_covariance_matrix() const;

            private:
                std::pair<real_t, real_t> drive_params;
                Eigen::Matrix<real_t, state_dimension, 1> control_matrix;
                Eigen::Matrix<real_t, state_dimension, state_dimension> covariance_matrix;
        }
    }
}

#endif
