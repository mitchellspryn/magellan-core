#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdexcept>
#include <unistd.h>

#include "contracts/detecting_sensor.hpp"
#include "contracts/image_obstacle.hpp"
#include "contracts/lidar_obstacle.hpp"
#include "contracts/obstacle.hpp"
#include "contracts/obstacle_type.hpp"
#include "contracts/perception_params.hpp"
#include "image_obstacle_detector.hpp"
#include "lidar_obstacle_detector.hpp"
#include "obstacle_merger.hpp"

int main(int argc, char** argv)
{
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
