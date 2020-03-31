#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace magellan
{
    namespace localization
    {
        class GlobalPose
        {
            public:
                Eigen::Vector3f GlobalPosition;
                Eigen::Quaternionf GlobalRotation;

                Eigen::Matrix<float, 3, 3> GlobalPositionCovariance;
                Eigen::Matrix<float, 4, 4> GlobalRotationCovariance;

                GlobalPose()
                {
                    this->GlobalPostion = Eigen::Vector3f::Zero();
                    this->GlobalRotation = Eigen::Quaternionf(1, 0, 0, 0);
                }
        };
    }
}
