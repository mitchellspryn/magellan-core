#ifndef GROUND_SEGMENTER_HPP
#define GROUND_SEGMENTER_HPP

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

class GroundSegmenter
{
    public:
        virtual ~GroundSegmenter() {};
        virtual Eigen::Matrix2i segment_ground(const sensor_msgs::PointCloud2 &stereo_point_cloud) = 0;
};

#endif
