#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include <unordered_map>
#include <sensor_msgs/PointCloud2.h>

#include "obstacle.hpp"

class ObstacleDetector 
{
    public:
        ObstacleDetector(const std::unordered_map<std::string, std::string> &configuration_parameters);

        void reconfigure(const std::unordered_map<std::string, std::string> &configuration_parameters);
        std::vector<Obstacle> detect(sensor_msgs::PointCloud2 stereo_camera_cloud, sensor_msgs::PointCloud2 laser_scan);

    private:


};

#endif
