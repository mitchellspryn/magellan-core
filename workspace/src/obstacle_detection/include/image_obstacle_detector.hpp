#ifndef IMAGE_OBSTACLE_DETECTOR_HPP
#define IMAGE_OBSTACLE_DETECTOR_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
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
        class ImageObstacleDetector
        {
            public:
                ImageObstacleDetector(const PerceptionParams &params);

                bool DetectObstacles(
                        const cv::Mat &input_image_rgb,  
                        const cv::Mat &input_image_depth,
                        std::vector<Obstacle> &output_obstacles);

                bool SetParameters(const PerceptionParams &params);

            private:
                PerceptionParams _params;
        };
    }
}

#endif
