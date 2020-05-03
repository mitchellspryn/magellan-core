#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgZedSensors.h>

const std::string g_global_frame_id = "map";
const std::string g_local_frame_id = "zed";

ros::Publisher g_zed_point_cloud_publisher;
ros::Publisher g_zed_image_publisher;
ros::Publisher g_zed_pose_publisher;

ros::Publisher g_rplidar_point_cloud_publisher;

tf::TransformBroadcaster g_transform_broadcaster;

void transform_zed_point_cloud(const sensor_msgs::PointCloud2::ConstPtr &incoming_msg)
{
    // Build the RGB image out of the point cloud
    sensor_msgs::Image zed_image_msg;
    zed_image_msg.encoding = "bgr8";
    zed_image_msg.width = incoming_msg->width;
    zed_image_msg.height = incoming_msg->height;
    zed_image_msg.header.stamp = incoming_msg->header.stamp;
    zed_image_msg.header.frame_id = g_local_frame_id;
    zed_image_msg.is_bigendian = false;
    zed_image_msg.step = 3 * incoming_msg->width;

    int num_pixels = zed_image_msg.width*zed_image_msg.height;
    zed_image_msg.data.resize(3*num_pixels);

    const unsigned char* point_cloud_data = incoming_msg->data.data();
    unsigned char* image_msg_data = &zed_image_msg.data[0];
    int num_points = incoming_msg->width * incoming_msg->height;

    point_cloud_data += 14;
    for (int i = 0; i < num_points; i++)
    {
        *image_msg_data++ = *(point_cloud_data--);
        *image_msg_data++ = *(point_cloud_data--);
        *image_msg_data++ = *(point_cloud_data);
        point_cloud_data += 18;
    }

    // Invert the Y axis for visualization in rviz
    sensor_msgs::PointCloud2 cloud_msg(*incoming_msg);
    
    float* point_cloud_data_float = reinterpret_cast<float*>(cloud_msg.data.data());
    point_cloud_data_float++;
    for (int i = 0; i < num_points; i++)
    {
        *point_cloud_data_float = *point_cloud_data_float * -1;
        point_cloud_data_float += cloud_msg.point_step / sizeof(float);
    }

    cloud_msg.header.frame_id = g_local_frame_id;

    g_zed_point_cloud_publisher.publish(cloud_msg);
    g_zed_image_publisher.publish(zed_image_msg);
}

void transform_zed_pose(const magellan_messages::MsgZedPose::ConstPtr &incoming_msg)
{
    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.header.stamp = incoming_msg->header.stamp;
    out_msg.header.frame_id = g_local_frame_id;

    out_msg.pose.pose.position = incoming_msg->pose.pose.position;
    out_msg.pose.pose.orientation = incoming_msg->pose.pose.orientation;

    // Invert pose along Y axis.
    // See here for the quaternion rotation logic: 
    // https://stackoverflow.com/questions/28673777/convert-quaternion-from-right-handed-to-left-handed-coordinate-system
    out_msg.pose.pose.position.y *= -1;
    out_msg.pose.pose.orientation.x *= -1;
    out_msg.pose.pose.orientation.z *= -1;

    // TODO: figure out how to change coordinate system for covariance matrix.
    // We're not computing it for now.
    // TODO: figure out how to draw confidence. If we want to.
    out_msg.pose.covariance[0] = -1;
    for (int i = 1; i < 36; i++)
    {
        out_msg.pose.covariance[i] = 0;
    }

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
                out_msg.pose.pose.position.x,
                out_msg.pose.pose.position.y,
                out_msg.pose.pose.position.z));

    transform.setRotation(tf::Quaternion(
                out_msg.pose.pose.orientation.x, 
                out_msg.pose.pose.orientation.y, 
                out_msg.pose.pose.orientation.z, 
                out_msg.pose.pose.orientation.w));

    g_transform_broadcaster.sendTransform(tf::StampedTransform(
                transform, out_msg.header.stamp, g_global_frame_id, g_local_frame_id));

    g_zed_pose_publisher.publish(out_msg.pose);
}

void transform_rplidar_scan(const sensor_msgs::PointCloud2::ConstPtr &incoming_msg)
{
    sensor_msgs::PointCloud2 output_msg(*incoming_msg);

    // Flip the scan over the Y axis
    float* point_cloud_data_float = reinterpret_cast<float*>(output_msg.data.data());
    point_cloud_data_float++;
    int num_points = output_msg.width * output_msg.height;
    for (int i = 0; i < num_points; i++)
    {
        *point_cloud_data_float = *point_cloud_data_float * -1; 
        point_cloud_data_float += output_msg.point_step / sizeof(float);
    }

    g_rplidar_point_cloud_publisher.publish(output_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magellan_rviz");

    ros::NodeHandle nh;
    
    g_zed_pose_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_zed_point_cloud", 10);
    g_zed_image_publisher = nh.advertise<sensor_msgs::Image>("output_zed_image", 10);
    g_zed_pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("output_zed_pose", 10);
    g_rplidar_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_rplidar_point_cloud", 10);

    ros::Subscriber zed_point_cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_zed_point_cloud", 10, transform_zed_point_cloud);
    ros::Subscriber zed_pose_subscriber = nh.subscribe<magellan_messages::MsgZedPose>("input_zed_pose", 10, transform_zed_pose);
    ros::Subscriber rplidar_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_rplidar_point_cloud", 10, transform_rplidar_scan);

    ros::spin();
}
