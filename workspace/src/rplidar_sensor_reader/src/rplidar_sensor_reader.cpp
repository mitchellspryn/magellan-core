#include "rplidar.h"

#include <limits>
#include <math.h>
#include <unistd.h>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <random>

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

sensor_msgs::PointField make_point_field(const std::string &name, const unsigned int offset, const unsigned int datatype)
{
    sensor_msgs::PointField pf;
    pf.count = 1;
    pf.datatype = datatype;
    pf.offset = offset;
    pf.name = name;
    return pf;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rplidar_sensor_reader");

    char usb_port_name[USB_PORT_MAX_LEN];
    char mode[RPLIDAR_MODE_NAME_MAX_LEN];

    float x_offset_m = 0;
    float y_offset_m = 0;
    float z_offset_m = 0;

    // Set the default 
    strcpy(mode, "stability");

    int c;
    int len;
    while((c = getopt(argc, argv, "n:m:x:y:z:")) != -1)
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
            case 'x':
                try
                {
                    x_offset_m = std::stof(optarg);
                }
                catch (...)
                {
                    ROS_ERROR("Could not convert string %s to float for x offset.",
                            optarg);
                    return -1;
                }
                break;
            case 'y':
                try
                {
                    y_offset_m = std::stof(optarg);
                }
                catch (...)
                {
                    ROS_ERROR("Could not convert string %s to float for y offset.",
                            optarg);
                    return -1;
                }
                break;
            case 'z':
                try
                {
                    z_offset_m = std::stof(optarg);
                }
                catch (...)
                {
                    ROS_ERROR("Could not convert string %s to float for z offset.",
                            optarg);
                    return -1;
                }
                break;
            default:
                ROS_ERROR("Unrecognized command line arg: %c", c);
                return -1;
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

    sensor_msgs::PointCloud2 point_cloud_msg;
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.is_dense = false;
    point_cloud_msg.header.frame_id = "map";

    int point_step = 16;
    point_cloud_msg.height = 1;
    point_cloud_msg.width = static_cast<int>(NUMBER_POINTS_PER_SCAN);
    point_cloud_msg.point_step = point_step;
    point_cloud_msg.row_step = point_step * point_cloud_msg.width;

    point_cloud_msg.fields.push_back(make_point_field("x", 0, 7)); //Float32
    point_cloud_msg.fields.push_back(make_point_field("y", 4, 7)); //Float32
    point_cloud_msg.fields.push_back(make_point_field("z", 8, 7)); //Float32
    point_cloud_msg.fields.push_back(make_point_field("intensity", 12, 7)); //Float32

    int data_size_in_bytes = point_step*point_cloud_msg.height*point_cloud_msg.width;
    point_cloud_msg.data.resize(data_size_in_bytes);

    float* point_cloud_data_ptr = reinterpret_cast<float*>(&point_cloud_msg.data[0]);

    for (int i = 0; i < 4*point_cloud_msg.width; i++)
    {
        point_cloud_data_ptr[i] = std::numeric_limits<float>::quiet_NaN();
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
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1000);

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

            for (int i = 0; i < 4*point_cloud_msg.width; i++)
            {
                point_cloud_data_ptr[i] = std::numeric_limits<float>::quiet_NaN();
            }

            for (size_t i = 0; i < node_count; i++)
            {
                // These formulas are blindly copied from the SDK
                int quality = nodes[i].quality;
                float distance = nodes[i].dist_mm_q2 / 4.0f;

                if (quality > 1 && distance > 0.01)
                {
                    float angle_degrees = nodes[i].angle_z_q14 * 90.0f / (1 << 14);
                    float angle_rad = angle_degrees * M_PI / 180.0f;
                
                    int index = (int)((angle_degrees / 360.0) * static_cast<float>(NUMBER_POINTS_PER_SCAN));

                    float* packet = point_cloud_data_ptr + (index * 4);

                    *packet++ = (-1.0f * cos(angle_rad)*distance / 1000.0f) + x_offset_m;
                    *packet++ = (-1.0f * sin(angle_rad)*distance / 1000.0f) + y_offset_m;
                    *packet++ = z_offset_m;
                    *packet = static_cast<float>(quality);
                }
            }

            ros::Time now = ros::Time::now();
            point_cloud_msg.header.stamp = now;
            
            publisher.publish(point_cloud_msg);
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
