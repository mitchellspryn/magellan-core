#include <math.h>
#include <unistd.h>
#include <stdexcept>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <magellan_messages/MsgSerialPortLine.h>
#include <magellan_messages/MsgMagellanImu.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#define PACKET_SEPARATOR '|'
#define GRAVITATIONAL_CONSTANT 9.81

ros::Publisher gps_publisher;
ros::Publisher imu_publisher;

ssize_t find_next_separator(const uint8_t* data, size_t pos, uint8_t separator, ssize_t length)
{
    ssize_t index = pos;

    while(index < length)
    {
        if (data[index] == separator)
        {
            return index;
        }

        index++;
    }

    return -1;
}

bool process_imu_packet(magellan_messages::MsgMagellanImu &imu_message, const uint8_t* data, size_t start_index, size_t end_index)
{
    int16_t x;
    int16_t y;
    int16_t z;

    constexpr double accel_scale = GRAVITATIONAL_CONSTANT * 2.0 / 32767.5;
    constexpr double gyro_scale = (250.0 / 32765.5) * (M_PI / 180.0);
    constexpr double mag_scale = 10.0 * 4192.0 / 8189.5;

    // Read accelerometer data
    x = (static_cast<int16_t>(data[start_index]) << 8) | static_cast<int16_t>(data[start_index + 1]);
    y = (static_cast<int16_t>(data[start_index + 2]) << 8) | static_cast<int16_t>(data[start_index + 3]);
    z = (static_cast<int16_t>(data[start_index + 4]) << 8) | static_cast<int16_t>(data[start_index + 5]);

    imu_message.imu.linear_acceleration.x = static_cast<double>(x) * accel_scale;
    imu_message.imu.linear_acceleration.y = static_cast<double>(y) * accel_scale;
    imu_message.imu.linear_acceleration.z = static_cast<double>(z) * accel_scale;

    // Read in temperature data
    x = (static_cast<int16_t>(data[start_index + 6]) << 8) | static_cast<int16_t>(data[start_index + 7]);

    // TODO: This came from sparkfun's website.
    // No idea if it's correct.
    imu_message.temperature = (static_cast<float>(x) / 333.87) + 21.0; 
    
    // Read in gyroscope data
    x = (static_cast<int16_t>(data[start_index + 8]) << 8) | static_cast<int16_t>(data[start_index + 9]);
    y = (static_cast<int16_t>(data[start_index + 10]) << 8) | static_cast<int16_t>(data[start_index + 11]);
    z = (static_cast<int16_t>(data[start_index + 12]) << 8) | static_cast<int16_t>(data[start_index + 13]);

    imu_message.imu.angular_velocity.x = static_cast<double>(x) * gyro_scale;
    imu_message.imu.angular_velocity.y = static_cast<double>(y) * gyro_scale;
    imu_message.imu.angular_velocity.z = static_cast<double>(z) * gyro_scale;

    // Read in magnetometer data
    // Magnetometer data is 14 bit 
    x = (static_cast<int16_t>(data[start_index + 14]) << 8) | static_cast<int16_t>(data[start_index + 15]);
    y = (static_cast<int16_t>(data[start_index + 16]) << 8) | static_cast<int16_t>(data[start_index + 17]);
    z = (static_cast<int16_t>(data[start_index + 18]) << 8) | static_cast<int16_t>(data[start_index + 19]);

    imu_message.magnetometer.x = static_cast<double>(x) * mag_scale;
    imu_message.magnetometer.y = static_cast<double>(y) * mag_scale;
    imu_message.magnetometer.z = static_cast<double>(z) * mag_scale;

    float length = (imu_message.magnetometer.x * imu_message.magnetometer.x) +
                    (imu_message.magnetometer.y * imu_message.magnetometer.y) +
                    (imu_message.magnetometer.z * imu_message.magnetometer.z);

    length = sqrt(length);

    // Convert magnetic XYZ into orientation
    Eigen::Vector3d base_vector(1, 0, 0);
    Eigen::Vector3d heading_vector(imu_message.magnetometer.x / length,
                                    imu_message.magnetometer.y / length,
                                    imu_message.magnetometer.z / length);

    Eigen::Quaterniond heading_quat = Eigen::Quaterniond::FromTwoVectors(base_vector, heading_vector);

    imu_message.imu.orientation.w = heading_quat.w();
    imu_message.imu.orientation.x = heading_quat.x();
    imu_message.imu.orientation.y = heading_quat.y();
    imu_message.imu.orientation.z = heading_quat.z();

    imu_message.imu.orientation_covariance[0] = -1;
    imu_message.imu.angular_velocity_covariance[0] = -1;
    imu_message.imu.linear_acceleration_covariance[0] = -1;

    return true;
}

bool check_nmea_checksum(const uint8_t* data, size_t start_index, size_t end_index)
{
    uint8_t checksum = 0;
    for (size_t i = start_index+1; i < end_index-3; i++)
    {
        checksum ^= data[i];
    }

    char dummy_buf[3];
    dummy_buf[0] = data[end_index-2];
    dummy_buf[1] = data[end_index-1];
    dummy_buf[2] = 0;

    uint8_t actual_checksum = std::stoi(dummy_buf, 0, 16);

    return (actual_checksum == checksum);
}

float gps_nmea_to_decimal(std::string nmea)
{
    size_t break_index = nmea.find('.');

    if ((break_index != std::string::npos) && (break_index >= 2))
    {
        break_index = break_index - 2;
    }
    else
    {
        break_index = 0;
    }

    std::string degrees = nmea.substr(0, break_index);
    std::string minutes = nmea.substr(break_index);

    float total_degrees = std::stof(degrees);
    float total_minutes = std::stof(minutes) / 60.0;

    return total_degrees + total_minutes;
}

bool process_gps_packet(sensor_msgs::NavSatFix &gps_message, const uint8_t* data, size_t start_index, size_t end_index)
{
    if (strncmp(reinterpret_cast<const char*>(data + start_index), "$GPGGA,", 7))
    {
        return false;
    }

    if (!check_nmea_checksum(data, start_index, end_index))
    {
        return false;
    }

    std::string message(data + start_index, data + end_index);
    std::stringstream ss(message);
    std::vector<std::string> split_message;
    split_message.reserve(16);

    while(ss.good())
    {
        std::string current_token;
        std::getline(ss, current_token, ',');
        split_message.push_back(current_token);
    }

    if (split_message[6][0] != '1' and split_message[6][0] != '2')
    {
        return false;
    }

    gps_message.latitude = gps_nmea_to_decimal(split_message[2]);
    gps_message.longitude = gps_nmea_to_decimal(split_message[4]);

    // TODO: use sea level or ellipsoid?
    gps_message.altitude = std::stof(split_message[9]);

    return true;
}

void message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data)
{
    bool imu_message_valid = false;
    bool gps_message_valid = false;

    if (input_data->data[0] != 0xFA)
    {
        ROS_ERROR("Incorrect packet header.");
        return;
    }

    sensor_msgs::NavSatFix gps_message;
    magellan_messages::MsgMagellanImu imu_message;

    size_t packet_start_index = 1;
    const uint8_t* data_start = &(input_data->data[0]);
    ssize_t packet_end_index = find_next_separator(data_start, 0, PACKET_SEPARATOR, input_data->data.size());

    while (packet_end_index > 0)
    {
        const uint8_t* current_packet = data_start + packet_start_index;        

        if (!strncmp(reinterpret_cast<const char*>(current_packet), "IMU:", 4))
        {
            imu_message_valid = process_imu_packet(imu_message, data_start, packet_start_index + 4, packet_end_index);
        }
        else if (!strncmp(reinterpret_cast<const char*>(current_packet), "GPS:", 4))
        {
            gps_message_valid = process_gps_packet(gps_message, data_start, packet_start_index + 4, packet_end_index);
        }

        packet_start_index = packet_end_index + 1;
        packet_end_index = find_next_separator(data_start, packet_start_index, PACKET_SEPARATOR, input_data->data.size());
    }

    ros::Time time = ros::Time::now();

    if (imu_message_valid)
    {
        imu_message.header.stamp = time;
        imu_publisher.publish(imu_message); 
    }

    if (gps_message_valid)
    {
        gps_message.header.stamp = time;
        gps_publisher.publish(gps_message);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduino_sensor_reader");

    ros::NodeHandle nh;
   
    gps_publisher = nh.advertise<sensor_msgs::NavSatFix>("output_topic_gps", 1000);
    imu_publisher = nh.advertise<magellan_messages::MsgMagellanImu>("output_topic_imu", 1000);

    ros::Subscriber subscriber = nh.subscribe<magellan_messages::MsgSerialPortLine>("input_topic", 1000, message_received_callback);

    ros::spin();
}
