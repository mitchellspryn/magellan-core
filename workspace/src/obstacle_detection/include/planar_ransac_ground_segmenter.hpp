#ifndef PLANAR_RANSAC_GROUND_SEGMENTER_HPP
#define PLANAR_RANSAC_GROUND_SEGMENTER_HPP

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

#include "ground_segmenter.hpp"

class PlanarRansacGroundSegmeneter : public GroundSegmenter
{
    public:
        virtual ~PlanarRansacGroundSegmeneter() {};
        virtual Eigen::Matrix2i segment_ground(const sensor_msgs::PointCloud2 &stereo_point_cloud);
};

#endif
