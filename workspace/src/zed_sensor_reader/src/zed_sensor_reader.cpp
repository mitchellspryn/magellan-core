#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <exception>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sl/Camera.hpp>
#include <std_msgs/ColorRGBA.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "magellan_messages/MsgZedPose.h"
#include "magellan_messages/MsgZedSensors.h"

ros::Publisher g_point_cloud_publisher;
ros::Publisher g_pose_publisher;
ros::Publisher g_sensors_publisher;

sl::Camera g_camera;
sl::RuntimeParameters g_runtime_parameters;

typedef struct capture_parameters
{
    bool publish_sensors;
    bool publish_pose;
    bool enable_area_memory;
    int frames_per_second;
} capture_parameters_t;

static std::string g_frame_id = "map";
static constexpr int g_pose_update_rate = 60;

inline void sl_vec3_to_ros_vec3(const sl::float3 &sl_data, geometry_msgs::Vector3 &ros_data)
{
    // These can't be memcpy
    // Vector3 is double, and sl::float3 is float
    ros_data.x = sl_data.x;
    ros_data.y = sl_data.y;
    ros_data.z = sl_data.z;
}

inline void sl_vec3_to_ros_point(const sl::float3 &sl_data, geometry_msgs::Point &point)
{
    point.x = sl_data.x;
    point.y = sl_data.y;
    point.z = sl_data.z;
}

inline void sl_quat_to_ros_quat(const sl::float4 &sl_data, geometry_msgs::Quaternion &quat)
{
    quat.w = sl_data.w;
    quat.x = sl_data.x;
    quat.y = sl_data.y;
    quat.z = sl_data.z;
}

inline void sl_cov_mat_to_ros_cov_mat(const sl::Matrix3f &sl_data, boost::array<double, 9> &ros_data)
{
    for (int i = 0; i < 9; i++)
    {
        ros_data[i] = sl_data.r[i];
    }
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

void initialize_point_cloud_msg(sensor_msgs::PointCloud2 &point_cloud_msg, const int height, const int width)
{
    int one = 1; // for endianness detection

    point_cloud_msg.height = height;
    point_cloud_msg.width = width;
    //point_cloud_msg.is_bigendian = !(*(char*)&one == 1); // TODO: does this actually work?
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.is_dense = true; // TODO: Is this right?

    int point_step = 16;
    point_cloud_msg.point_step = point_step;
    point_cloud_msg.row_step = point_step * width;

    point_cloud_msg.fields.push_back(make_point_field("x", 0, 7)); //Float32
    point_cloud_msg.fields.push_back(make_point_field("y", 4, 7));
    point_cloud_msg.fields.push_back(make_point_field("z", 8, 7));
    point_cloud_msg.fields.push_back(make_point_field("r", 12, 2)); //Uint8
    point_cloud_msg.fields.push_back(make_point_field("g", 13, 2)); 
    point_cloud_msg.fields.push_back(make_point_field("b", 14, 2)); 
    point_cloud_msg.fields.push_back(make_point_field("a", 15, 2)); 

    point_cloud_msg.data.resize(point_step * width * height);

    //point_cloud_msg.header.frame_id = g_frame_id;
    point_cloud_msg.header.frame_id = "zed";

    for (int i = 0; i < point_step*width*height; i++)
    {
        point_cloud_msg.data[i] = 0;
    }
}

void image_pose_grab_thread(const capture_parameters_t &capture_parameters)
{
    sl::Mat point_cloud;
    sl::Pose pose;

    sensor_msgs::PointCloud2 point_cloud_msg;
    magellan_messages::MsgZedPose pose_msg;
    
    pose_msg.header.frame_id = g_frame_id;
    pose_msg.header.frame_id = g_frame_id;

    const int height = 720;
    const int width = 1280;
    
    initialize_point_cloud_msg(point_cloud_msg, height, width);

    pose_msg.pose.covariance[0] = -1;
    pose_msg.twist.covariance[0] = -1;
    for (int i = 1; i < 36; i++)
    {
        pose_msg.pose.covariance[i] = 0;
    }
    pose_msg.confidence = -1;


    const int frame_downsample_counter = g_pose_update_rate / capture_parameters.frames_per_second;
    int frame_counter = 0;

    while (ros::ok())
    {
        // TODO: how much to sleep?
        if (g_camera.grab(g_runtime_parameters) == sl::ERROR_CODE::SUCCESS)
        {
            ros::Time now = ros::Time::now();

            if (frame_counter == 0)
            {
                point_cloud_msg.header.stamp = now;
                g_camera.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
                memcpy(static_cast<void*>(&point_cloud_msg.data[0]), point_cloud.getPtr<sl::float4>(), height*width*4*sizeof(float));
                g_point_cloud_publisher.publish(point_cloud_msg);
            }

            if (capture_parameters.publish_pose)
            {
                pose_msg.header.stamp = now;
                g_camera.getPosition(pose, sl::REFERENCE_FRAME::WORLD);

                sl_vec3_to_ros_point(pose.getTranslation(), pose_msg.pose.pose.position);
                sl_quat_to_ros_quat(pose.getOrientation(), pose_msg.pose.pose.orientation);

                pose_msg.twist.twist.linear.x  = pose.twist[0];
                pose_msg.twist.twist.linear.y  = pose.twist[1];
                pose_msg.twist.twist.linear.z  = pose.twist[2];
                pose_msg.twist.twist.angular.x = pose.twist[3];
                pose_msg.twist.twist.angular.y = pose.twist[4];
                pose_msg.twist.twist.angular.z = pose.twist[5];

                // TODO: Should this be separate message? Or should we publish marker array?
                // For now putting confidence in xx-yy-zz of covariance
                // Note that pose_confidence is backwards - 100 == full confidence, 0 == no confidence
                //pose_msg.pose.covariance[0] = 100.0 - pose.pose_confidence;
                //pose_msg.pose.covariance[7] = 100.0 - pose.pose_confidence;
                //pose_msg.pose.covariance[14] = 100.0 -pose.pose_confidence;
                
                if (capture_parameters.enable_area_memory)
                {
                    pose_msg.confidence = pose.pose_confidence;
                }
                
                for (int i = 0; i < 36; i++)
                {
                    pose_msg.pose.covariance[i] = static_cast<double>(pose.pose_covariance[i]);
                    pose_msg.twist.covariance[i] = static_cast<double>(pose.twist_covariance[i]);
                }
                
                g_pose_publisher.publish(pose_msg);
            }

            frame_counter++;
            frame_counter %= frame_downsample_counter;
        }

        ros::spinOnce();
    }
}

// TODO: should we use timers instead of threads?
// It would be cleaner, but worried about performance implications
void sensor_grab_thread(const capture_parameters_t &capture_parameters)
{
    sl::SensorsData sensors_data;
    sl::Timestamp last_collected_timestamp = 0;

    magellan_messages::MsgZedSensors zed_sensor_msg;
    zed_sensor_msg.header.frame_id = g_frame_id;

    for (int i = 0; i < 9; i++)
    {
        zed_sensor_msg.imu.angular_velocity_covariance[i] = 0;
        zed_sensor_msg.imu.linear_acceleration_covariance[i] = 0;
        zed_sensor_msg.imu.orientation_covariance[i] = 0;
        zed_sensor_msg.magnetic_field.magnetic_field_covariance[i] = 0;
    }

    while (ros::ok())
    {
        // TODO: how much to sleep?
        if (g_camera.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS)
        {
            if (sensors_data.imu.timestamp > last_collected_timestamp)
            {
                zed_sensor_msg.header.stamp = ros::Time::now();

                sl_vec3_to_ros_vec3(sensors_data.imu.angular_velocity, zed_sensor_msg.imu.angular_velocity);
                sl_cov_mat_to_ros_cov_mat(sensors_data.imu.angular_velocity_covariance, zed_sensor_msg.imu.angular_velocity_covariance);

                sl_vec3_to_ros_vec3(sensors_data.imu.linear_acceleration, zed_sensor_msg.imu.linear_acceleration);
                sl_cov_mat_to_ros_cov_mat(sensors_data.imu.linear_acceleration_covariance, zed_sensor_msg.imu.linear_acceleration_covariance);

                sl_vec3_to_ros_vec3(sensors_data.magnetometer.magnetic_field_calibrated, zed_sensor_msg.magnetic_field.magnetic_field);

                zed_sensor_msg.pressure = sensors_data.barometer.pressure;
                zed_sensor_msg.temperature = sensors_data.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU];
                zed_sensor_msg.relative_altitude = sensors_data.barometer.relative_altitude;

                g_sensors_publisher.publish(zed_sensor_msg);
            }
        }

        // TODO: is getSensorsData() blocking?
        usleep(1000);
    }
}

sl::ERROR_CODE init_camera(const capture_parameters_t &parameters)
{
    sl::InitParameters init_parameters;
    init_parameters.camera_fps = 60; // Maximum accuracy for pose calculations. Will downsample for point cloud.
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_minimum_distance = 0.2;
    init_parameters.depth_maximum_distance = 20;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.depth_stabilization = true;
    init_parameters.enable_image_enhancement = true;
    init_parameters.enable_right_side_measure = false;
    init_parameters.sensors_required = (parameters.publish_pose || parameters.publish_sensors);

    g_runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;

    return g_camera.open(init_parameters);
}

sl::ERROR_CODE init_positional_tracking(const capture_parameters_t &parameters)
{
    if (!parameters.publish_pose)
    {
        return sl::ERROR_CODE::SUCCESS;
    }

    sl::PositionalTrackingParameters position_tracking_parameters;
    position_tracking_parameters.enable_area_memory = parameters.enable_area_memory;
    position_tracking_parameters.enable_imu_fusion = true;
    position_tracking_parameters.enable_pose_smoothing = true;
    position_tracking_parameters.set_floor_as_origin = false;

    return g_camera.enablePositionalTracking(position_tracking_parameters);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_sensor_reader");

    capture_parameters_t capture_parameters;
    capture_parameters.frames_per_second = 50;
    capture_parameters.publish_pose = false;
    capture_parameters.publish_sensors = false;
    capture_parameters.enable_area_memory = true;

    // TODO: args
    int c;
    while ((c = getopt(argc, argv, "dpsf:")) != -1)
    {
        switch (c)
        {
            case 'd':
                capture_parameters.enable_area_memory = false;
                break;
            case 'p':
                capture_parameters.publish_pose = true;
                break;
            case 's':
                capture_parameters.publish_sensors = true;
                break;
            case 'f':
                try
                {
                    capture_parameters.frames_per_second = std::stoi(optarg, NULL);
                }
                catch (std::exception ex)
                {
                    ROS_FATAL("Could not parse frames per second from provided string %s. The following error was encountered: %s", 
                            optarg, 
                            ex.what());
                    return 1;
                }
                break;
            default:
                ROS_FATAL("Unknown command line flag: %c.", c);
                return 1;
        }
    }

    if (g_pose_update_rate % capture_parameters.frames_per_second != 0)
    {
        ROS_FATAL("Error: Requested FPS of %d is not a factor of %d.",
                capture_parameters.frames_per_second,
                g_pose_update_rate);
        return 1;
    }


    sl::ERROR_CODE open_result = init_camera(capture_parameters);
    if (open_result != sl::ERROR_CODE::SUCCESS)
    {
        ROS_FATAL("Could not open camera. Open returns %d.", static_cast<int>(open_result));
        return 1;
    }

    if (capture_parameters.publish_pose)
    {
        sl::ERROR_CODE position_tracking_err_code = init_positional_tracking(capture_parameters);
        if (position_tracking_err_code != sl::ERROR_CODE::SUCCESS)
        {
            ROS_FATAL("Could not initialize positional tracking. Init function returns %d.", static_cast<int>(position_tracking_err_code));

            if (g_camera.isPositionalTrackingEnabled())
            {
                g_camera.disablePositionalTracking();
            }
            
            if (g_camera.isOpened())
            {
                g_camera.close();
            }

            return 1;
        }
    }

    ros::NodeHandle nh;

    g_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_topic_point_cloud", 1000);

    if (capture_parameters.publish_pose)
    {
        g_pose_publisher = nh.advertise<magellan_messages::MsgZedPose>("output_topic_pose", 1000);
    }

    if (capture_parameters.publish_sensors)
    {
        g_sensors_publisher = nh.advertise<magellan_messages::MsgZedSensors>("output_topic_sensors", 1000);
    }
    
    std::thread image_publish_thread(image_pose_grab_thread, capture_parameters);
    std::thread sensor_publish_thread;

    if (capture_parameters.publish_sensors)
    {
        sensor_publish_thread = std::thread(sensor_grab_thread, capture_parameters);
    }

    ros::spin();

    image_publish_thread.join();

    if (capture_parameters.publish_sensors)
    {
        sensor_publish_thread.join();
    }

    if (capture_parameters.publish_pose && g_camera.isPositionalTrackingEnabled())
    {
        g_camera.disablePositionalTracking();
    }

    if (g_camera.isOpened())
    {
        g_camera.close();
    }

    return 0;
}
