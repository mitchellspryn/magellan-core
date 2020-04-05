#include "contracts/global_pose.hpp"

namespace magellan
{
    namespace localization
    {
        GlobalPose::GlobalPose()
        {
            this->global_position.setZero();
            this->global_rotation = quaternionr_t(1, 0, 0, 0);
            this->global_position_covariance.setZero();
            this->global_rotation_covariance.setZero();
        }
    }
}

