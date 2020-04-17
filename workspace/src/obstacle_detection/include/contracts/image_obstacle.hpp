#ifndef IMAGE_OBSTACLE_HPP
#define IMAGE_OBSTACLE_HPP

#include <opencv2/core/types.hpp>
#include <vector>

#include "contracts/detecting_sensor.hpp"
#include "contracts/obstacle.hpp"
#include "contracts/obstacle_type.hpp"

namespace magellan
{
    namespace obstacle_detection
    {
        class ImageObstacle : public Obstacle
        {
            public:
                ImageObstacle(const std::vector<cv::Point3f> &real_world_location,
                        const DetectingSensor &detecting_sensor,
                        const ObstacleType &obstacle_type,
                        const cv::RotatedRect &image_location,
                        const long &pixel_count) :
                    Obstacle(real_world_location, detecting_sensor, obstacle_type),
                    _image_location(image_location),
                    _pixel_count(pixel_count) {}

                const cv::RotatedRect &get_image_location() const { return this->_image_location; }
                const long &get_pixel_count() const { return this->_pixel_count; }

            private:
                cv::RotatedRect _image_location;
                long _pixel_count;
        };
    }
}

#endif
