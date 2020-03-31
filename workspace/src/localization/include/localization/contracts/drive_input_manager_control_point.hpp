#ifndef DRIVE_INPUT_MANAGER_CONTROL_POINT_HPP
#define DRIVE_INPUT_MANAGER_CONTROL_POINT_HPP

#include <eigen3/Eigen/Dense>
#include <utility>

#include "constants.hpp"

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

                void get_drive_params(std::pair<real_t, real_t> &drive_params);
                void get_control_matrix(Eigen::Matrix<real_t, state_dimension, 1> &control_matrix);
                void get_covariance_matrix(Eigen::Matrix<real_t, state_dimension, state_dimension> &covariance_matrix);

            private:
                std::pair<real_t, real_t> drive_params;
                Eigen::Matrix<real_t, state_dimension, 1> control_matrix;
                Eigen::Matrix<real_t, state_dimension, state_dimension> covariance_matrix;
        }
    }
}

#endif
