#include "contracts/global_state.cpp"

namespace ml = magellan::localization;

ml::GlobalPose::GlobalPose()
{
    this->GlobalPosition.setZero();
    this->GlobalRotation = quaternionr_t(1, 0, 0, 0);
    this->GlobalPositionCovariance.setZero();
    this->GlobalRotationCovariance.setZero();
}
