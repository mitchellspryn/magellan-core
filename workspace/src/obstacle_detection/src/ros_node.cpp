#include <mutex>
#include "ros/ros.h"
#include <stdexcept>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <unistd.h>

#include <magellan_messages/MsgObstacleDetection.h>
#include <magellan_messages/MsgObstacleDetectorConfig.h>
#include "../include/obstacle_detector.hpp"
#include "ros/time.h"

ros::Publisher detection_publisher;
ros::Publisher debug_point_cloud_publisher;

bool publish_debug_point_cloud = false;
bool have_cloud = false;
float prediction_hz = 10;

ObstacleDetector detector;
std::mutex detection_mutex;

sensor_msgs::PointCloud2::ConstPtr latest_stereo_point_cloud;
sensor_msgs::PointCloud2 debug_point_cloud;
sensor_msgs::PointCloud2 dummy_cloud;

magellan_messages::MsgObstacleDetection obstacle_detection_result;

void stereo_point_cloud_received_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    detection_mutex.lock();
    latest_stereo_point_cloud = cloud;
    have_cloud = true;
    detection_mutex.unlock();
}

void rplidar_point_cloud_received_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    // TODO: when the algorithm decides it needs it, we can set it.
    // For now, avoid thrashing the mutex lock.
}

void obstacle_detection_configuration_received_callback(const magellan_messages::MsgObstacleDetectorConfig::ConstPtr &config)
{
    detection_mutex.lock();
    detector.set_internal_parameters(*config);
    detection_mutex.unlock();
}

void detection_thread(const ros::TimerEvent &event)
{
    if (!have_cloud)
    {
        return;
    }

    detection_mutex.lock();

    bool detection_successful = detector.detect(*latest_stereo_point_cloud, dummy_cloud, obstacle_detection_result);
    if (detection_successful && publish_debug_point_cloud)
    {
        debug_point_cloud = detector.debug_annotate(*latest_stereo_point_cloud); 
    }

    detection_mutex.unlock();

    ros::Time now = ros::Time::now();
    if (detection_successful)
    {
        obstacle_detection_result.header.frame_id = "zed";
        obstacle_detection_result.header.stamp = now;
        detection_publisher.publish(obstacle_detection_result);
    }

    if (detection_successful && publish_debug_point_cloud)
    {
        debug_point_cloud.header.frame_id = "zed";
        debug_point_cloud.header.stamp = now;
        debug_point_cloud_publisher.publish(debug_point_cloud);
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "obstacle_detection");

    int c;
    while ((c = getopt(argc, argv, "df:")) != -1)
    {
        switch (c)
        {
            case 'd':
                publish_debug_point_cloud = true;
                break;
            case 'f':
                try
                {
                    prediction_hz = std::stof(optarg);
                    break;
                }
                catch (std::exception ex)
                {
                    ROS_FATAL("Could not parse prediction frequency from string %s. Got the following error: %s",
                            optarg, 
                            ex.what());
                    return 1;
                }
            default:
                std::string error = "Unrecotnized argument: " + std::to_string(static_cast<char>(c)) + ".";
                throw std::runtime_error(error);
        }
    }

    ros::NodeHandle nh;

    detection_publisher = nh.advertise<magellan_messages::MsgObstacleDetection>("output_topic_detection", 1000);

    if (publish_debug_point_cloud)
    {
        debug_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_topic_debug_point_cloud", 1000);
    }

    ros::Subscriber stereo_point_cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_topic_stereo_point_cloud", 1, stereo_point_cloud_received_callback);
    ros::Subscriber rplidar_point_cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_topic_rplidar_point_cloud", 1, rplidar_point_cloud_received_callback);
    ros::Subscriber config_subscriber = nh.subscribe<magellan_messages::MsgObstacleDetectorConfig>("input_config", 1, obstacle_detection_configuration_received_callback);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / prediction_hz), detection_thread);
    timer.start();

    ros::spin();
}
