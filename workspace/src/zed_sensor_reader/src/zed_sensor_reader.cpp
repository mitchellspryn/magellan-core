#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include <exception>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sl/Camera.hpp>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include <magellan_messages/MsgZedImagePose.h>
#include <magellan_messages/MsgZedSensors.h>

ros::Publisher image_pose_publisher;
ros::Publisher sensor_publisher;
sl::Camera camera;
sl::RuntimeParameters runtime_parameters;

typedef struct capture_parameters
{
    bool publish_depth_image;
    bool publish_point_cloud;
    bool publish_sensors;
    bool publish_pose;
    int frames_per_second;
} capture_parameters_t;

inline void sl_vec3_to_ros_vec3(const sl::float3 &sl_data, geometry_msgs::Vector3 &ros_data)
{
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

inline void sl_cov_mat_to_ros_cov_mat(const sl::Matrix3f &sl_data, boost::array<double, 9> ros_data)
{
    for (int i = 0; i < 9; i++)
    {
        ros_data[i] = sl_data.r[i];
    }
}

// Copied and lightly modified from sl_tools.
// see imageToROSmsg here:
// https://github.com/stereolabs/zed-ros-wrapper/blob/master/zed_nodelets/src/tools/src/sl_tools.cpp
void copy_mat_to_ros_imgmsg(sensor_msgs::Image &img_msg, const sl::Mat &img, const ros::Time &t) 
{
    img_msg.header.stamp = t;
    img_msg.header.frame_id = "";
    img_msg.height = img.getHeight();
    img_msg.width = img.getWidth();

    int num = 1; // for endianness detection
    img_msg.is_bigendian = !(*(char*)&num == 1);

    img_msg.step = img.getStepBytes();

    size_t size = img_msg.step * img_msg.height;
    img_msg.data.resize(size);

    sl::MAT_TYPE dataType = img.getDataType();

    switch (dataType) 
    {
        case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
            img_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::float1>(), size);
            break;

        case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
            img_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::float2>(), size);
            break;

        case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
            img_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::float3>(), size);
            break;

        case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
            img_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::float4>(), size);
            break;

        case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
            img_msg.encoding = sensor_msgs::image_encodings::MONO8;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::uchar1>(), size);
            break;

        case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
            img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::uchar2>(), size);
            break;

        case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
            img_msg.encoding = sensor_msgs::image_encodings::BGR8;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::uchar3>(), size);
            break;

        case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
            img_msg.encoding = sensor_msgs::image_encodings::BGRA8;
            memcpy((char*)(&img_msg.data[0]), img.getPtr<sl::uchar4>(), size);
            break;
    }
}


// TODO: should we use timers instead of threads?
// It would be cleaner, but worried about performance implications
void sensor_grab_thread(const capture_parameters_t &capture_parameters)
{
    sl::SensorsData sensors_data;
    sl::Timestamp last_collected_timestamp = -1;

    magellan_messages::MsgZedSensors zed_sensor_message;

    while (ros::ok())
    {
        // TODO: how much to sleep?
        if (camera.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS)
        {
            if (sensors_data.imu.timestamp > last_collected_timestamp)
            {
                zed_sensor_message.header.stamp = ros::Time::now();

                sl_vec3_to_ros_vec3(sensors_data.imu.angular_velocity, zed_sensor_message.imu.angular_velocity);
                sl_cov_mat_to_ros_cov_mat(sensors_data.imu.angular_velocity_covariance, zed_sensor_message.imu.angular_velocity_covariance);

                sl_vec3_to_ros_vec3(sensors_data.imu.linear_acceleration, zed_sensor_message.imu.linear_acceleration);
                sl_cov_mat_to_ros_cov_mat(sensors_data.imu.linear_acceleration_covariance, zed_sensor_message.imu.linear_acceleration_covariance);

                sl_vec3_to_ros_vec3(sensors_data.magnetometer.magnetic_field_calibrated, zed_sensor_message.magnetometer);
                
                zed_sensor_message.pressure = sensors_data.barometer.pressure;
                zed_sensor_message.temperature = sensors_data.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::BAROMETER];

                sensor_publisher.publish(zed_sensor_message);
            }
        }
    }
}

void image_pose_grab_thread(const capture_parameters_t &capture_parameters)
{
    sl::Mat rgb;
    sl::Mat depth;
    sl::Mat point_cloud;
    sl::Pose pose;

    magellan_messages::MsgZedImagePose image_pose_msg;

    image_pose_msg.has_depth_image = capture_parameters.publish_depth_image;
    image_pose_msg.has_point_cloud = capture_parameters.publish_point_cloud;
    image_pose_msg.has_pose = capture_parameters.publish_pose;

    while (ros::ok())
    {
        // TODO: how much to sleep?
        if (camera.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
        {
            image_pose_msg.header.stamp = ros::Time::now();

            // TODO: if we need more performance, we can special-case copy_mat_to_ros_imgmsg to remove some computation
            // e.g. if we want to avoid recomputing the size of the image each frame. 
            camera.retrieveImage(rgb, sl::VIEW::LEFT);
            copy_mat_to_ros_imgmsg(image_pose_msg.rgb, rgb, image_pose_msg.header.stamp);

            if (capture_parameters.publish_depth_image)
            {
                camera.retrieveMeasure(depth, sl::MEASURE::DEPTH);
                copy_mat_to_ros_imgmsg(image_pose_msg.depth, depth, image_pose_msg.header.stamp);
            }

            if (capture_parameters.publish_point_cloud)
            {
                camera.retrieveMeasure(depth, sl::MEASURE::XYZRGBA);

                // TODO: copy to point cloud struct
            }

            if (capture_parameters.publish_pose)
            {
                camera.getPosition(pose, sl::REFERENCE_FRAME::WORLD);

                sl_vec3_to_ros_point(pose.getTranslation(), image_pose_msg.pose.position);
                sl_quat_to_ros_quat(pose.getOrientation(), image_pose_msg.pose.orientation);
            }

            image_pose_publisher.publish(image_pose_msg);
        }
    }
}

sl::ERROR_CODE init_camera(const capture_parameters_t &parameters)
{
    sl::InitParameters init_parameters;
    init_parameters.camera_fps = parameters.frames_per_second;
    init_parameters.camera_resolution = sl::RESOLUTION::VGA;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_parameters.coordinate_units = sl::UNIT::MILLIMETER;
    init_parameters.depth_minimum_distance = 200;
    init_parameters.depth_maximum_distance = 1000 * 20;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.depth_stabilization = true;
    init_parameters.enable_image_enhancement = true;
    init_parameters.enable_right_side_measure = false;
    init_parameters.sensors_required = (parameters.publish_pose || parameters.publish_sensors);

    runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;

    return camera.open(init_parameters);
}

sl::ERROR_CODE init_positional_tracking(const capture_parameters_t &parameters)
{
    if (!parameters.publish_pose)
    {
        return sl::ERROR_CODE::SUCCESS;
    }

    sl::PositionalTrackingParameters position_tracking_parameters;
    position_tracking_parameters.enable_area_memory = true;
    position_tracking_parameters.enable_imu_fusion = true;
    position_tracking_parameters.enable_pose_smoothing = true;
    position_tracking_parameters.set_floor_as_origin = false;

    return camera.enablePositionalTracking(position_tracking_parameters);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_sensor_reader");

    capture_parameters_t capture_parameters;
    capture_parameters.frames_per_second = 60;
    capture_parameters.publish_depth_image = false;
    capture_parameters.publish_point_cloud = false;
    capture_parameters.publish_pose = false;
    capture_parameters.publish_sensors = false;

    // TODO: args
    int c;
    while ((c = getopt(argc, argv, "dposf:")) != -1)
    {
        switch (c)
        {
            case 'd':
                capture_parameters.publish_depth_image = true;
                break;
            case 'p':
                capture_parameters.publish_point_cloud = true;
                break;
            case 'o':
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
            default:
                ROS_FATAL("Unknown command line flag: %c.", c);
                return 1;
        }
    }

    sl::ERROR_CODE open_result = init_camera(capture_parameters);
    if (open_result != sl::ERROR_CODE::SUCCESS)
    {
        ROS_FATAL("Could not open camera. Open returns %d.", static_cast<int>(open_result));
        return 1;
    }

    sl::ERROR_CODE position_tracking_err_code = init_positional_tracking(capture_parameters);
    if (position_tracking_err_code != sl::ERROR_CODE::SUCCESS)
    {
        ROS_FATAL("Could not initialize positional tracking. Init function returns %d.", static_cast<int>(position_tracking_err_code));

        if (camera.isPositionalTrackingEnabled())
        {
            camera.disablePositionalTracking();
        }
        
        if (camera.isOpened())
        {
            camera.close();
        }

        return 1;
    }

    ros::NodeHandle nh;
    image_pose_publisher = nh.advertise<magellan_messages::MsgZedImagePose>("output_topic_image_pose", 1000);
    image_pose_publisher = nh.advertise<magellan_messages::MsgZedSensors>("output_topic_sensors", 1000);

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

    if (camera.isPositionalTrackingEnabled())
    {
        camera.disablePositionalTracking();
    }

    if (camera.isOpened())
    {
        camera.close();
    }

    return 0;
}
