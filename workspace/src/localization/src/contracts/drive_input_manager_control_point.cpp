#include "contracts/drive_input_manager_control_point.hpp"

namespace magellan
{
    namespace localization
    {
        DriveInputManagerControlPoint::DriveInputManagerControlPoint(
            std::pair<real_t, real_t>drive_params,
            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> control_matrix,
            Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> covariance_matrix)
        {

        }

        const std::pair<real_t, real_t>& DriveInputManagerControlPoint::get_drive_params() const
        {
            return this->drive_params;
        }

        const Eigen::Matrix<real_t, state_dimension, 1>& DriveInputManagerControlPoint::get_control_matrix() const
        {
            return this->control_matrix;
        }

        const Eigen::Matrix<real_t, state_dimension, state_dimension>& DriveInputManagerControlPoint::get_covariance_matrix() const
        {
            return this->covariance_matrix;
        }
    }
}

