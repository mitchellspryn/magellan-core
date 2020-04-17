#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <opencv2/core/types.hpp>
#include <vector>

#include "contracts/detecting_sensor.hpp"
#include "contracts/obstacle_type.hpp"

namespace magellan
{
    namespace obstacle_detection
    {
        class Obstacle
        {
            public:
                const std::vector<cv::Point3f> &get_real_world_pixels() const { return this->_real_world_location; } 
                const DetectingSensor &get_detecting_sensor() const { return this->_detecting_sensor; }
                const ObstacleType &get_obstacle_type() const { return this->_obstacle_type; }

            protected:
                Obstacle(const std::vector<cv::Point3f> &real_world_location, 
                        const DetectingSensor &detecting_sensor, 
                        const ObstacleType &obstacle_type) :
                    _real_world_location(real_world_location), 
                    _detecting_sensor(detecting_sensor),
                    _obstacle_type(obstacle_type) {};

                std::vector<cv::Point3f> _real_world_location;
                DetectingSensor _detecting_sensor;
                ObstacleType _obstacle_type;
        };
    }
}

#endif
