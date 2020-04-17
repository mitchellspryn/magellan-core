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
        class LidarObstacle : public Obstacle
        {
            public:
                LidarObstacle(const std::vector<cv::Point3f> &real_world_location,
                        const DetectingSensor &detecting_sensor,
                        const ObstacleType &obstacle_type,
                        const float &left_angle, 
                        const float &right_angle,
                        const long &num_points) :
                    Obstacle(real_world_location, detecting_sensor, obstacle_type),
                    _left_angle(left_angle), 
                    _right_angle(right_angle),
                    _num_points(num_points) {}

                const float &get_left_angle() const { return this->_left_angle; }
                const float &get_max_angle() const { return this->_right_angle; }
                const long &get_num_points() const { return this->_num_points; }

            private:
                float _left_angle;
                float _right_angle;
                long _num_points;
        };
    }
}

#endif
