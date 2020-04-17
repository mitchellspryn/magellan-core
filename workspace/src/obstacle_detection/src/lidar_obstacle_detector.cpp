#include "lidar_obstacle_detector.hpp"
#include "sensor_msgs/LaserScan.h"

namespace mo = magellan::obstacle_detection;

mo::LidarObstacleDetector::LidarObstacleDetector(const PerceptionParams &params)
    : _params(params)
{
    
}

bool mo::LidarObstacleDetector::DetectObstacles(
        const sensor_msgs::LaserScan::ConstPtr &laser_scan,
        std::vector<mo::Obstacle> &output_obstacles)
{
    return true;
}

bool mo::LidarObstacleDetector::SetParameters(const PerceptionParams &other)
{
    return true;
}
