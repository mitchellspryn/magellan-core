#include "rplidar.h"

#include <math.h>
#include <unistd.h>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace rplidar = rp::standalone::rplidar;

#define USB_PORT_MAX_LEN 24
#define RPLIDAR_MODE_NAME_MAX_LEN 64
#define RPLIDAR_BAUD 256000
#define RPLIDAR_NODE_BUF_SIZE 8192
#define NUMBER_POINTS_PER_SCAN 460

bool should_continue = false;
rplidar::RPlidarDriver* driver = NULL;

bool strcmpci(const char* a, const char* b)
{
    while ((*a) && (*b))
    {
        if (tolower(*a) != tolower(*b))
        {
            return false;
        }
        a++;
        b++;
    }

    return !( (*a) || (*b) );
}

void clean_up(int dummy_signal)
{
    // NOTE: the RPLidar library appears to do some funky signal handling of its own.
    // Attempting to clean up within the signal handler does not work. 
    should_continue = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rplidar_sensor_reader");

    char usb_port_name[USB_PORT_MAX_LEN];
    char mode[RPLIDAR_MODE_NAME_MAX_LEN];

    // Set the default 
    strcpy(mode, "stability");

    int c;
    int len;
    while((c = getopt(argc, argv, "n:m:")) != -1)
    {
        switch (c)
        {
            case 'n':
                len = strnlen(optarg, USB_PORT_MAX_LEN + 1);
                if (len > USB_PORT_MAX_LEN)
                {
                    ROS_ERROR("The USB port name is too long.");
                    return -1;
                }

                strncpy(usb_port_name, optarg, USB_PORT_MAX_LEN);
                break;
            case 'm':
                len = strnlen(optarg, RPLIDAR_MODE_NAME_MAX_LEN + 1);
                if (len > RPLIDAR_MODE_NAME_MAX_LEN)
                {
                    ROS_ERROR("The mode name is too long.");
                    return -1;
                }

                strncpy(mode, optarg, RPLIDAR_MODE_NAME_MAX_LEN);
                break;
        }
    }

    if (!strcmpci(mode, "stability")
        && !strcmpci(mode, "boost")
        && !strcmpci(mode, "sensitivity"))
    {
        ROS_ERROR("Unsupported mode passed with -m flag: %s.", mode);
        ROS_ERROR("Supported modes are stability (default), boost, sensitivity.");
        return -1;
    }

    rplidar::RPlidarDriver* driver = rplidar::RPlidarDriver::CreateDriver(rplidar::DRIVER_TYPE_SERIALPORT);

    if (!driver)
    {
        ROS_ERROR("Could not create the driver.");
        return -1;
    }

    if (IS_FAIL(driver->connect(usb_port_name, RPLIDAR_BAUD)))
    {
        ROS_ERROR("Could not connect to lidar on port %s at baud %d", usb_port_name, RPLIDAR_BAUD);
        return -1;
    }

    // Check for lidar health
    rplidar_response_device_health_t healthinfo;
    if (IS_FAIL(driver->getHealth(healthinfo)))
    {
        ROS_ERROR("Could not get the health status of the device.");
        return -1;
    }

    if (healthinfo.status != RPLIDAR_STATUS_OK)
    {
        ROS_ERROR("Lidar health check reports status %d, error code %d.", 
                healthinfo.status,
                healthinfo.error_code);
        return -1;
    }

    std::vector<rplidar::RplidarScanMode> supported_modes;
    rplidar::RplidarScanMode chosen_mode;
    driver->getAllSupportedScanModes(supported_modes);
    bool mode_found = false;

    for (size_t i = 0; i < supported_modes.size(); i++) 
    {
        if (strcmpci(supported_modes[i].scan_mode, mode))
        {
            chosen_mode = supported_modes[i];
            mode_found = true;
            break;
        }
    }

    if (!mode_found)
    {
        ROS_ERROR("Could not get mode %s from lidar.", mode);
        return -1;
    }

    int numberOfPointsPerScan = NUMBER_POINTS_PER_SCAN;

    sensor_msgs::LaserScan laser_scan;
    laser_scan.angle_min = 0;
    laser_scan.angle_max = 2 * M_PI;
    laser_scan.angle_increment = (2.0 * M_PI) / numberOfPointsPerScan;
    laser_scan.range_min = 200;
    laser_scan.range_max = 25000;
    
    for (int i = 0; i < numberOfPointsPerScan; i++)
    {
        laser_scan.ranges.push_back(0);
        laser_scan.intensities.push_back(0);
    }

    driver->startMotor();

    if (IS_FAIL(driver->startScan(0, 0, 0, &chosen_mode)))
    {
        ROS_ERROR("Could not start the scan. Driver->startScan returned unsuccessfully.");
        driver->stopMotor();
        rplidar::RPlidarDriver::DisposeDriver(driver);
        return -1;
    }

    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<sensor_msgs::LaserScan>("output_topic", 1000);

    ros::Rate loop_rate(15);
    uint32_t result;
    rplidar_response_measurement_node_hq_t nodes[RPLIDAR_NODE_BUF_SIZE];

    signal(SIGINT, clean_up);
    should_continue = true;
    while(ros::ok() && should_continue)
    {
        size_t node_count = RPLIDAR_NODE_BUF_SIZE;
        result = driver->grabScanDataHq(nodes, node_count);
        if (IS_FAIL(result))
        {
            ROS_ERROR("Dropped a scan from the lidar. Result code is %d.", result);
        }
        else
        {
            driver->ascendScanData(nodes, node_count);

            std::fill(laser_scan.ranges.begin(), laser_scan.ranges.end(), 0);
            std::fill(laser_scan.intensities.begin(), laser_scan.intensities.end(), 0);

            // Debug variables
            int numFilled = 0;
            int maxIdx = 0;
            for (size_t i = 0; i < node_count; i++)
            {
                // These formulas are blindly copied from the SDK
                // Not sure if they are right...
                float distance = nodes[i].dist_mm_q2 / 4.0f;
                float angle_degrees = nodes[i].angle_z_q14 * 90.0f / (1 << 14);
                int quality = nodes[i].quality;

                if (quality > 1)
                {
                    int index = (int)((angle_degrees / 360.0) * numberOfPointsPerScan);
                    laser_scan.ranges[index] = distance;
                    laser_scan.intensities[index] = quality;

                    maxIdx = std::max(maxIdx, index);
                    numFilled++;
                }
            }

            ros::Time now = ros::Time::now();
            laser_scan.header.stamp = now;
            
            publisher.publish(laser_scan);
        }

        if (should_continue)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    driver->stop();
    driver->stopMotor();
    rplidar::RPlidarDriver::DisposeDriver(driver);
    return 0;
}
