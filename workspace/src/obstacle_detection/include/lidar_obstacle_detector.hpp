#ifndef LIDAR_OBSTACLE_DETECTOR_HPP
#define LIDAR_OBSTACLE_DETECTOR_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <sensor_msgs/LaserScan.h>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "contracts/detecting_sensor.hpp"
#include "contracts/image_obstacle.hpp"
#include "contracts/obstacle.hpp"
#include "contracts/obstacle_type.hpp"
#include "contracts/perception_params.hpp"

namespace magellan
{
    namespace obstacle_detection
    {
        class LidarObstacleDetector
        {
            public:
                LidarObstacleDetector(const PerceptionParams &params);

                bool DetectObstacles(
                        const sensor_msgs::LaserScan::ConstPtr &laser_scan,  
                        std::vector<Obstacle> &output_obstacles);

                bool SetParameters(const PerceptionParams &params);

            private:
                PerceptionParams _params;
        };
    }
}

#endif
