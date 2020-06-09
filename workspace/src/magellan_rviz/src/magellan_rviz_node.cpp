#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <vector>

#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgZedSensors.h>
#include <magellan_messages/MsgObstacleDetection.h>

const std::string g_global_frame_id = "map";
const std::string g_local_frame_id = "zed";

std::unordered_set<int> g_cone_ids_sent;

ros::Publisher g_obstacle_detection_cone_marker_publisher;
ros::Publisher g_obstacle_detection_nav_publisher;
ros::Publisher g_obstacle_debug_point_cloud_publisher;
ros::Publisher g_zed_point_cloud_publisher;
ros::Publisher g_zed_image_publisher;
ros::Publisher g_zed_pose_publisher;

ros::Publisher g_rplidar_point_cloud_publisher;

// this needs to be initialized after ros::init() has been called
std::unique_ptr<tf::TransformBroadcaster> g_transform_broadcaster = nullptr;

typedef struct MinMaxBoundsAggregator
{
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();

    void add(int x, int y)
    {
        this->min_x = std::min(this->min_x, x);
        this->min_y = std::min(this->min_y, y);
        this->max_x = std::max(this->max_x, x);
        this->max_y = std::max(this->max_y, y);
    }

} MinMaxBoundsAggregator_t;

sensor_msgs::PointField make_point_field(const std::string &name, const unsigned int offset, const unsigned int datatype)
{
    sensor_msgs::PointField pf;
    pf.count = 1;
    pf.datatype = datatype;
    pf.offset = offset;
    pf.name = name;
    return pf;
}

void get_cone_rgba(visualization_msgs::Marker &marker, int cone_id)
{
    marker.color.a = 1;

    if (cone_id == 1)
    {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
    }
    else if (cone_id == 2)
    {
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
    }
    else if (cone_id == 3)
    {
        marker.color.r = 0;
        marker.color.g = 165.0 / 255.0;
        marker.color.b = 1;
    }
    else if (cone_id == 4)
    {
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 1;
    }
    else
    {
        throw std::runtime_error("Unexpected cone_id: " + std::to_string(cone_id));
    }
}

void transform_obstacle_detection(const magellan_messages::MsgObstacleDetection::ConstPtr &incoming_msg)
{
    int width = incoming_msg->map_metadata.width;
    int height = incoming_msg->map_metadata.height;
    int num_blocks = width * height;
    nav_msgs::OccupancyGrid grid;
    grid.header = incoming_msg->header;
    grid.info = incoming_msg->map_metadata;
    grid.data.resize(num_blocks);

    std::unordered_map<int, MinMaxBoundsAggregator_t> cone_bounds;

    float ysub = grid.info.origin.position.y;
    float grid_max_y = grid.info.origin.position.y + (grid.info.resolution * grid.info.height);
    float grid_min_y = grid_max_y * -1;
    grid.info.origin.position.y = grid_min_y;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int incoming_msg_idx = (y * width) + x;
            int grid_idx = ((height - 1 - y) * width) + x;

            if (incoming_msg->matrix[incoming_msg_idx] == -3 || incoming_msg->matrix[incoming_msg_idx] == -2)
            {
                grid.data[grid_idx] = -1;
            }
            else if (incoming_msg->matrix[incoming_msg_idx] == -1)
            {
                grid.data[grid_idx] = 100;
            }
            else if (incoming_msg->matrix[incoming_msg_idx] == 0)
            {
                grid.data[grid_idx] = 0;
            }
            else
            {
                grid.data[incoming_msg_idx] = 100;
                int8_t cone_id = incoming_msg->matrix[incoming_msg_idx];
                cone_bounds[cone_id].add(x, y);
            }
        }
    }
        
    std::vector<visualization_msgs::Marker> markers;
    for (const auto &cone_kvp : cone_bounds)
    {
        int cone_id = cone_kvp.first;
        MinMaxBoundsAggregator_t cone_bounds = cone_kvp.second;

        visualization_msgs::Marker marker;
        marker.header = incoming_msg->header;
        marker.header.frame_id = "zed";
        marker.ns = "cones";
        marker.id = cone_id;
        marker.type = 1;
        marker.action = 0;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        get_cone_rgba(marker, cone_id);
        marker.frame_locked = true;

        marker.scale.x = static_cast<float>(cone_bounds.max_x - cone_bounds.min_x + 1) * incoming_msg->map_metadata.resolution;
        marker.scale.y = static_cast<float>(cone_bounds.max_y - cone_bounds.min_y + 1) * incoming_msg->map_metadata.resolution;
        marker.scale.z = 1;

        marker.pose.position.x = ( (static_cast<float>((cone_bounds.max_x + cone_bounds.min_x)) / 2.0f) * incoming_msg->map_metadata.resolution ) 
            + incoming_msg->map_metadata.origin.position.x;
        marker.pose.position.y = ( (static_cast<float>((cone_bounds.max_y + cone_bounds.min_y) / 2.0f)) * -incoming_msg->map_metadata.resolution );
        marker.pose.position.y -= ysub;
        marker.pose.position.z = incoming_msg->map_metadata.origin.position.z;

        markers.push_back(marker);
        g_cone_ids_sent.insert(cone_id);
    }

    std::vector<int> cones_to_remove;
    for (const auto &present_cone : g_cone_ids_sent)
    {
        if (cone_bounds.count(present_cone) == 0)
        {
            visualization_msgs::Marker marker;
            marker.header = incoming_msg->header;
            marker.ns = "cones";
            marker.id = present_cone;
            marker.type = 1;
            marker.action = 2;

            markers.push_back(marker);
            cones_to_remove.push_back(present_cone);
        }
    }

    for (const auto &cone : cones_to_remove)
    {
        g_cone_ids_sent.erase(cone);
    }

    visualization_msgs::MarkerArray cone_array;
    cone_array.markers = markers;

    g_obstacle_detection_nav_publisher.publish(grid);

    if (markers.size() > 0)
    {
        g_obstacle_detection_cone_marker_publisher.publish(cone_array);
    }
}

void transform_obstacle_debug_point_cloud(const sensor_msgs::PointCloud2::ConstPtr &incoming_msg)
{
    int num_points = incoming_msg->width * incoming_msg->height;
    sensor_msgs::PointCloud2 cloud_msg(*incoming_msg);

    cloud_msg.fields.clear();
    cloud_msg.fields.push_back(make_point_field("x", 0, 7)); //Float32
    cloud_msg.fields.push_back(make_point_field("y", 4, 7));
    cloud_msg.fields.push_back(make_point_field("z", 8, 7));
    cloud_msg.fields.push_back(make_point_field("nx", 12, 7));
    cloud_msg.fields.push_back(make_point_field("ny", 16, 7));
    cloud_msg.fields.push_back(make_point_field("nz", 20, 7));
    cloud_msg.fields.push_back(make_point_field("confidence", 24, 7)); 
    cloud_msg.fields.push_back(make_point_field("rgb", 28, 7));
    
    // Invert the Y axis for visualization in rviz
    float* point_cloud_data_float = reinterpret_cast<float*>(cloud_msg.data.data());
    point_cloud_data_float++;
    for (int i = 0; i < num_points; i++)
    {
        *point_cloud_data_float = *point_cloud_data_float * -1;
        point_cloud_data_float += cloud_msg.point_step / sizeof(float);
    }

    cloud_msg.header.frame_id = g_local_frame_id;

    g_obstacle_debug_point_cloud_publisher.publish(cloud_msg);
}

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

    point_cloud_data += 28;
    for (int i = 0; i < num_points; i++)
    {
        *image_msg_data++ = *point_cloud_data++;
        *image_msg_data++ = *point_cloud_data++;
        *image_msg_data++ = *point_cloud_data;
        point_cloud_data += 30;
    }

    sensor_msgs::PointCloud2 cloud_msg(*incoming_msg);

    cloud_msg.fields.clear();
    cloud_msg.fields.push_back(make_point_field("x", 0, 7)); //Float32
    cloud_msg.fields.push_back(make_point_field("y", 4, 7));
    cloud_msg.fields.push_back(make_point_field("z", 8, 7));
    cloud_msg.fields.push_back(make_point_field("nx", 12, 7));
    cloud_msg.fields.push_back(make_point_field("ny", 16, 7));
    cloud_msg.fields.push_back(make_point_field("nz", 20, 7));
    cloud_msg.fields.push_back(make_point_field("confidence", 24, 7)); 
    cloud_msg.fields.push_back(make_point_field("rgb", 28, 7));
    
    // Invert the Y axis for visualization in rviz
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
    out_msg.header.frame_id = g_global_frame_id;

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
    for (int i = 0; i < 36; i++)
    {
        out_msg.pose.covariance[i] = incoming_msg->pose.covariance[i];
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

    if (g_transform_broadcaster == nullptr) {
        g_transform_broadcaster = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    }

    g_transform_broadcaster->sendTransform(tf::StampedTransform(
                transform, out_msg.header.stamp, g_global_frame_id, g_local_frame_id));

    g_zed_pose_publisher.publish(out_msg);
}

void transform_rplidar_scan(const sensor_msgs::PointCloud2::ConstPtr &incoming_msg)
{
    sensor_msgs::PointCloud2 output_msg(*incoming_msg);

    output_msg.header.frame_id = g_local_frame_id;

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
    
    g_obstacle_detection_cone_marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("output_obstacle_detection_cone_viz", 10);
    g_obstacle_detection_nav_publisher = nh.advertise<nav_msgs::OccupancyGrid>("output_obstacle_detection_occupancy_matrix", 10);
    g_obstacle_debug_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_obstacle_detection_debug_point_cloud", 10);
    g_zed_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_zed_point_cloud", 10);
    g_zed_image_publisher = nh.advertise<sensor_msgs::Image>("output_zed_image", 10);
    g_zed_pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("output_zed_pose", 10);
    g_rplidar_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_rplidar_point_cloud", 10);

    ros::Subscriber zed_point_cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_zed_point_cloud", 10, transform_zed_point_cloud);
    ros::Subscriber obstacle_detection_debug_point_cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_obstacle_detection_debug_point_cloud", 10, transform_obstacle_debug_point_cloud);
    ros::Subscriber obstacle_detection_detection_subscriber = nh.subscribe<magellan_messages::MsgObstacleDetection>("input_obstacle_detection", 10, transform_obstacle_detection);
    ros::Subscriber zed_pose_subscriber = nh.subscribe<magellan_messages::MsgZedPose>("input_zed_pose", 10, transform_zed_pose);
    ros::Subscriber rplidar_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input_rplidar_point_cloud", 10, transform_rplidar_scan);

    ros::spin();
}
