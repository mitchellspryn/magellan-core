#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class GeometryUtils
{
    public:
        static geometry_msgs::Point RotateByQuaternion(
            const geometry_msgs::Point &p,
            const geometry_msgs::Quaternion &q);

        static geometry_msgs::Quaternion HamiltonProduct(
            const geometry_msgs::Quaternion &q1,
            const geometry_msgs::Quaternion &q2);
};

#endif
