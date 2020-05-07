#include "obstacle_detector.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <unordered_map>

ObstacleDetector::ObstacleDetector(const std::unordered_map<std::string, std::string> &configuration_parameters)
{

}

void ObstacleDetector::reconfigure(const std::unordered_map<std::string, std::string> &configuration_parameters)
{

}

std::vector<Obstacle> detect(sensor_msgs::PointCloud2 stereo_camera_cloud, sensor_msgs::PointCloud2 laser_scan)
{

}
