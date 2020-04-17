#include "image_obstacle_detector.hpp"
#include "contracts/perception_params.hpp"

namespace mo = magellan::obstacle_detection;

mo::ImageObstacleDetector::ImageObstacleDetector(const mo::PerceptionParams &params)
    : _params("blah")
{
}

bool mo::ImageObstacleDetector::DetectObstacles(
        const cv::Mat &input_image_rgb,
        const cv::Mat &input_image_depth,
        std::vector<Obstacle> &output_obstacles)
{
    return true;
}

bool mo::ImageObstacleDetector::SetParameters(const PerceptionParams &params)
{
    return true;
}
