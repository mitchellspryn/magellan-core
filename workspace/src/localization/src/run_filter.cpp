#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
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
#include "rosbag/message_instance.h"

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

void recompute_filter_and_publish(const ros::Time current_time)
{
    if (filter != nullptr)
    {
        magellan_messages::MsgMagellanLocalization output_msg;
        output_msg.header.stamp = current_time;

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

void load_imu_messages(
        const ros::Time window_start_time, 
        const ros::Time window_end_time,
        const std::vector<std::string> &imu_topic_name,
        const rosbag::Bag &bag)
{
    rosbag::View imu_view;
    imu_view.addQuery(bag, window_start_time, window_end_time);

    for (rosbag::View::const_iterator i = imu_view.begin(); i != imu_view.end(); ++i)
    {
        auto msg = i->instantiate<magellan_messages::MsgMagellanImu>();

        // View will return start and end point, inclusive.
        // Filter to avoid double counting
        if (msg.get()->header.stamp < window_end_time)
        {
            filter->accept_imu_message(msg);
        }
    }
}

void load_gps_messages(
        const ros::Time window_start_time, 
        const ros::Time window_end_time,
        const std::vector<std::string> &gps_topic_name,
        const rosbag::Bag &bag)
{
    rosbag::View gps_view;
    gps_view.addQuery(bag, window_start_time, window_end_time);

    for (rosbag::View::const_iterator i = gps_view.begin(); i != gps_view.end(); ++i)
    {
        auto msg = i->instantiate<sensor_msgs::NavSatFix>();

        // View will return start and end point, inclusive.
        // Filter to avoid double counting
        if (msg.get()->header.stamp < window_end_time)
        {
            filter->accept_gps_message(msg);
        }
    }
}

void load_drive_messages(
        const ros::Time window_start_time, 
        const ros::Time window_end_time,
        const std::vector<std::string> &drive_topic_name,
        const rosbag::Bag &bag)
{
    rosbag::View drive_view;
    drive_view.addQuery(bag, window_start_time, window_end_time);

    for (rosbag::View::const_iterator i = drive_view.begin(); i != drive_view.end(); ++i)
    {
        auto msg = i->instantiate<magellan_messages::MsgMagellanDrive>();

        // View will return start and end point, inclusive.
        // Filter to avoid double counting
        if (msg.get()->header.stamp < window_end_time)
        {
            filter->accept_drive_message(msg);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_test_node");

    std::string filter_config_path;
    std::string bag_path;
    std::string imu_topic_name;
    std::string gps_topic_name;
    std::string drive_topic_name;
    float update_hz;

    int c;
    while((c = getopt(argc, argv, "f:u:b:i:g:d:")) != -1)
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
            case 'b':
                bag_path = std::string(optarg);
                break;
            case 'i':
                imu_topic_name = std::string(optarg);
                break;
            case 'g':
                gps_topic_name = std::string(optarg);
                break;
            case 'd':
                drive_topic_name = std::string(optarg);
                break;
            default:
                ROS_FATAL("Unrecognized argument %c.", c);
                return 1;
        }
    }

    ros::NodeHandle nh;

    output_publisher = nh.advertise<magellan_messages::MsgMagellanLocalization>("output_topic", 1000);

    filter = std::make_shared<ml::LinearKalmanFilter>(filter_config_path);

    std::vector<std::string> topics_to_include {imu_topic_name, gps_topic_name, drive_topic_name};
    std::vector<std::string> imu_topic_name_vec {imu_topic_name};
    std::vector<std::string> gps_topic_name_vec {gps_topic_name};
    std::vector<std::string> drive_topic_name_vec {drive_topic_name};

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    ros::Duration update_dt(1.0 / update_hz);
    
    try
    {
        rosbag::View all_view(bag, rosbag::TopicQuery(topics_to_include));

        ros::Time start_time = all_view.getBeginTime();
        ros::Time end_time = all_view.getEndTime();

        ros::Time window_start_time = start_time;
        ros::Time window_end_time = window_start_time + update_dt;

        while(window_start_time < end_time)
        {
            load_imu_messages(window_start_time, window_end_time, imu_topic_name_vec, bag);
            load_gps_messages(window_start_time, window_end_time, gps_topic_name_vec, bag);
            load_drive_messages(window_start_time, window_end_time, drive_topic_name_vec, bag);
            
            recompute_filter_and_publish(window_end_time);

            window_start_time += update_dt;
            window_end_time = window_start_time + update_dt;
        }
    }
    catch (std::exception ex)
    {
        if (bag.isOpen())
        {
            bag.close();
        }

        throw;
    }
}
