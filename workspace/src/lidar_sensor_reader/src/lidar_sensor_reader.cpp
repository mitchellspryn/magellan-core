#include <math.h>
#include <unistd.h>
#include <stdexcept>
#include <chrono>

#include <ros/ros.h>
#include <magellan_messages/MsgSerialPortLine.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan lidar_scan;
ros::Publisher publisher;
ros::Subscriber subscriber;
int last_index  = -1;
int num_messages_received = 0;
int min_num_messages_needed = 0;
std::chrono::steady_clock::time_point scan_start_time;
bool scan_start_time_point_initialized = false;

void reset_lidar_scan()
{
    lidar_scan.angle_min = 0;
    lidar_scan.angle_max = 2 * M_PI;
    lidar_scan.angle_increment = 1.0 * M_PI / 180.0;
    lidar_scan.range_min = 150;
    lidar_scan.range_max = 6000;

    for (unsigned int i = 0; i < 360; i++)
    {
        lidar_scan.ranges[i] = 0;
        lidar_scan.intensities[i] = 0;
    }
}

uint32_t compute_checksum(const std::vector<uint8_t> &data)
{
  uint32_t data_list[10];
  for (int i = 0; i < 10; i++)
  {
    data_list[i] = data[2*i] + (data[(2*i)+1] << 8);
  }

  uint32_t checksum_tmp = 0;
  for (int i = 0; i < 10; i++)
  {
    checksum_tmp = (checksum_tmp << 1) + data_list[i];
  }

  uint32_t checksum = (checksum_tmp & 0x7FFF) + (checksum_tmp >> 15);
  return checksum & 0x7FFF;
}

bool packet_valid(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data)
{
    if (input_data->data.size() != 22)
    {
        ROS_WARN("Received message of unexpected size in Lidar Reader. Expected 22 bytes, got %ld bytes",
                input_data->data.size()
        );

        return false;
    }

    if (input_data->data[0] != 0xFA)
    {
        ROS_WARN("Received message with invalid start packet in Lidar Reader. Expected 0xFA, got 0x%x.",
                input_data->data[0]
        );

        return false;
    }

    uint32_t computed_checksum = compute_checksum(input_data->data);
    uint32_t actual_checksum = input_data->data[20] + ((input_data->data[21]) << 8);

    if (computed_checksum != actual_checksum)
    {
        ROS_WARN("Received message with invalid checksum. Expected 0x%x, got 0x%x.",
                actual_checksum,
                computed_checksum
        );

        return false;
    }

    uint8_t packet_index = input_data->data[1];
    if (packet_index < 0xA0 || packet_index > 0xF9)
    {
        ROS_WARN("Received message with invalid packet index. Got 0x%x, which is outside the range 0xA0 - 0xF9",
                packet_index
        );
        
        return false;
    }

    return true;
}

void message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data)
{
    if (!packet_valid(input_data))
    {
        return;
    }

    if (!scan_start_time_point_initialized)
    {
        scan_start_time_point_initialized = true;
        scan_start_time = std::chrono::steady_clock::now();
    }

    uint8_t packet_index = input_data->data[1] - 0xA0;

    if (packet_index < last_index)
    {
        if (num_messages_received > min_num_messages_needed)
        {
            std::chrono::steady_clock::time_point scan_end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> scan_time = scan_end_time - scan_start_time;

            lidar_scan.scan_time = scan_time.count();

            // TODO: Should this be 90, as 4 points come back with each scan?
            lidar_scan.time_increment = scan_time.count() / 360.0;

            publisher.publish(lidar_scan);
        }
        else
        {
            ROS_WARN("Scan wrapped around, but did not receive required number of chunks. Needed %d to publish, but only received %d.",
                    num_messages_received,
                    min_num_messages_needed
            );
        }

        reset_lidar_scan();
        num_messages_received = 0;
        scan_start_time = std::chrono::steady_clock::now();
    }

    // TODO: do something with the speed information in bytes 2, 3.
    // Speed = data[2] + data[3] << 8
    // in 64ths of an RPM.
    
    for (int i = 0; i < 4; i++)
    {
        // TODO: do something with the range error (bit 7) and strength error (bit 6) flags in range_upper
        uint8_t range_lower = input_data->data[4 + (4*i)];     
        uint8_t range_upper = input_data->data[5 + (4*i)] & 0x3F;
        uint8_t intensity_lower = input_data->data[6 + (4*i)];
        uint8_t intensity_upper = input_data->data[7 + (4*i)];

        lidar_scan.ranges[(packet_index*4) + i] = (range_upper << 8) + range_lower;
        lidar_scan.intensities[(packet_index*4) + i] = (intensity_upper << 8) + intensity_lower;
    }

    num_messages_received++;
    last_index = packet_index;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_sensor_reader");

    int c;
    while((c = getopt(argc, argv, "m:")) != -1)
    {
        switch (c)
        {
            case 'm':
                try
                {
                    min_num_messages_needed = std::stol(optarg);     
                }
                catch (std::exception ex)
                {
                    ROS_FATAL("Could not parse min_num_messages_needed parameter from string %s. The string was not able to be interpered as a long.",
                            optarg
                    );
                    return 1;
                }

        }
    }

    for (unsigned int i = 0; i < 360; i++)
    {
        lidar_scan.ranges.push_back(0);
        lidar_scan.intensities.push_back(0);
    }

    reset_lidar_scan();
    
    ros::NodeHandle nh;
    publisher = nh.advertise<sensor_msgs::LaserScan>("output_topic", 1000);
    subscriber = nh.subscribe<magellan_messages::MsgSerialPortLine>("input_topic", 1000, message_received_callback);

    ros::spin();
}
