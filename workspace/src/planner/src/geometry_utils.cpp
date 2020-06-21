#include "../include/geometry_utils.hpp"

geometry_msgs::Point GeometryUtils::RotateByQuaternion(
    const geometry_msgs::Point &p,
    const geometry_msgs::Quaternion &q)
{
    geometry_msgs::Quaternion n_q;
    n_q.w = q.w;
    n_q.x = -q.x;
    n_q.y = -q.y;
    n_q.z = -q.z;

    geometry_msgs::Quaternion point_q;
    point_q.w = 0;
    point_q.x = p.x;
    point_q.y = p.y;
    point_q.z = p.z;

    geometry_msgs::Quaternion out_q = 
        GeometryUtils::HamiltonProduct(
            GeometryUtils::HamiltonProduct(q, point_q), n_q);

    geometry_msgs::Point out;
    out.x = out_q.x;
    out.y = out_q.y;
    out.z = out_q.z;

    return out;
}

geometry_msgs::Quaternion GeometryUtils::HamiltonProduct(
    const geometry_msgs::Quaternion &q1, 
    const geometry_msgs::Quaternion &q2)
{
    geometry_msgs::Quaternion result;

    result.w = ((q1.w*q2.w) - (q1.x*q2.x) - (q1.y*q2.y) - (q2.z*q2.z));
    result.x = ((q1.w*q2.x) + (q1.x*q2.w) + (q1.y*q2.z) - (q2.z*q2.y));
    result.y = ((q1.w*q2.y) - (q1.x*q2.z) + (q1.y*q2.w) + (q2.z*q2.x));
    result.z = ((q1.w*q2.z) - (q1.x*q2.y) - (q1.y*q2.x) - (q2.z*q2.w));

    return result;
}
