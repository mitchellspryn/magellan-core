#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <magellan_messages/MsgMagellanDrive.h>
#include <magellan_messages/MsgMagellanImu.h>
#include <magellan_messages/MsgMagellanLocalization.h>
#include <sensor_msgs/NavSatFix.h>
#include <unistd.h>
#include <stdexcept>
#include <ctype.h>
#include <geometry_msgs/Point32.h>
#include <magellan_messages/MsgMagellanRotationf.h>

#include "contracts/global_pose.hpp"
#include "filters/filter.hpp"
#include "filters/linear_kalman_filter.hpp"
#include "ros/time.h"
#include "definitions.hpp"

namespace ml = magellan::localization;

std::shared_ptr<ml::Filter> filter = nullptr;
ros::Publisher output_publisher;
ros::Time last_time;

inline void assign_point_32(geometry_msgs::Point32 &point, const ml::vector3r_t &vec)
{
    point.x = vec[0];
    point.y = vec[1];
    point.z = vec[2];
}

inline void assign_rpy(magellan_messages::MsgMagellanRotationf &point, const ml::vector3r_t &vec)
{
    point.roll = vec[0];
    point.pitch = vec[1];
    point.yaw = vec[2];
}

inline void assign_covariance(boost::array<float, 36> &covariance, const Eigen::Matrix<ml::real_t, 3, 3> &cov)
{
    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 3; x++)
        {
            covariance[(y*3)+x] = cov(y, x);
        }
    }
}

void recompute_filter_and_publish(const ros::TimerEvent &e)
{
    if (filter != nullptr)
    {
        magellan_messages::MsgMagellanLocalization output_msg;
        output_msg.header.stamp = ros::Time::now();

        ros::Duration dt = output_msg.header.stamp - last_time;
        last_time = output_msg.header.stamp;

        filter->update_global_pose(dt);

        ml::GlobalPose pose = filter->get_global_pose();

        assign_point_32(output_msg.position, pose.global_position);
        assign_point_32(output_msg.velocity, pose.global_velocity);
        assign_point_32(output_msg.acceleration, pose.global_acceleration);

        assign_rpy(output_msg.heading, pose.global_rotation);
        assign_rpy(output_msg.angular_velocity, pose.global_angular_velocity);

        assign_covariance(output_msg.position_covariance, pose.global_position_covariance);
        assign_covariance(output_msg.velocity_covariance, pose.global_velocity_covariance);
        assign_covariance(output_msg.acceleration_covariance, pose.global_acceleration_covariance);

        assign_covariance(output_msg.heading_covariance, pose.global_rotation_covariance);
        assign_covariance(output_msg.angular_velocity_covariance, pose.global_angular_velocity_covariance);

        output_publisher.publish(output_msg);
    }
}

void imu_message_received_callback(const magellan_messages::MsgMagellanImu::ConstPtr &msg)
{
    if (filter != nullptr)
    {
        filter->accept_imu_message(msg);
    }
}

void gps_message_received_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (filter != nullptr)
    {
        filter->accept_gps_message(msg);
    }
}

void drive_message_received_callback(const magellan_messages::MsgMagellanDrive::ConstPtr &msg)
{
    if (filter != nullptr)
    {
        filter->accept_drive_message(msg);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_node");

    std::string filter_config_path;
    float update_hz;

    int c;
    while((c = getopt(argc, argv, "f:u:")) != -1)
    {
        switch (c)
        {
            case 'f':
                filter_config_path = std::string(optarg);
                break;
            case 'u':
                try
                {
                    update_hz = std::stof(optarg, NULL);
                }
                catch (std::exception ex)
                {
                    ROS_FATAL(
                            "Could not parse update rate from provided string %s. The following error was encountered: %s.",
                            optarg,
                            ex.what());
                    return 1;
                }
                break;
            default:
                ROS_FATAL("Unrecognized argument %c.", c);
                return 1;
        }
    }

    ros::NodeHandle nh;

    output_publisher = nh.advertise<magellan_messages::MsgMagellanLocalization>("output_topic", 1000);

    ros::Subscriber imu_subscriber = nh.subscribe<magellan_messages::MsgMagellanImu>("input_imu", 1, imu_message_received_callback);
    ros::Subscriber gps_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("input_gps", 1, gps_message_received_callback);
    ros::Subscriber drive_subscriber = nh.subscribe<magellan_messages::MsgMagellanDrive>("input_drive", 1, drive_message_received_callback);

    filter = std::make_shared<ml::LinearKalmanFilter>(filter_config_path);
    last_time = ros::Time::now();

    ros::Timer publishTimer = nh.createTimer(ros::Duration(1.0 / update_hz), recompute_filter_and_publish);

    ros::spin();
}
