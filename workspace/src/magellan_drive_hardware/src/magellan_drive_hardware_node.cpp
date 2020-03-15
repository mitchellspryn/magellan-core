#include <math.h>
#include <signal.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <unistd.h>

#include <ros/ros.h>
#include <magellan_messages/MsgSerialPortLine.h>
#include <magellan_messages/MsgMagellanDrive.h>

#define CMD_M1_FORWARD         0
#define CMD_M1_BACKWARD        1
#define CMD_M2_FORWARD         4
#define CMD_M2_BACKWARD        5
#define CMD_SET_M1_MAX_CURRENT 133
#define CMD_SET_M2_MAX_CURRENT 134
#define CMD_GET_M1_MAX_CURRENT 135
#define CMD_GET_M2_MAX_CURRENT 136
#define LEFT_ROBOCLAW_ADDRESS  130
#define RIGHT_ROBOCLAW_ADDRESS 128

#define MOTOR_CURRENT_LIMIT_AMPS 30

ros::Publisher left_motor_controller_publisher;
ros::Publisher right_motor_controller_publisher;

static std::vector<uint8_t> left_response;
static bool left_response_valid = false;
static std::vector<uint8_t> right_response;
static bool right_response_valid = false;

void left_serial_message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data);
void right_serial_message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data);
void command_message_received_callback(const magellan_messages::MsgMagellanDrive::ConstPtr &input_data);
void sigint_handler(int signal);
void drive(double left_throttle, double right_throttle);
bool drive_motor(uint8_t address, uint8_t command_id, uint8_t value, size_t retires, bool &verify_flag, std::vector<uint8_t> &received_data, ros::Publisher &publisher);
bool verify_motor_speed_command_received(bool &verify_flag, std::vector<uint8_t> &received_data, size_t usec_timeout);
void send_command(uint8_t address, uint8_t command_id, uint8_t *data, size_t data_len, bool &verify_flag, ros::Publisher &publisher);
void append_crc(std::vector<uint8_t> &data);
void set_motor_current_limit(uint8_t motor_address, uint8_t write_command_id, uint8_t read_command_id, uint32_t max_amps, bool &verify_flag, std::vector<uint8_t> &received_data, ros::Publisher &publisher);
bool initialize_motor_controllers();

void left_serial_message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data)
{
    if (input_data->data.size() > 0)
    {
        left_response.reserve(input_data->data.size());
        std::copy(input_data->data.begin(), input_data->data.end(), std::back_inserter(left_response));
        left_response_valid = true;
    }
}

void right_serial_message_received_callback(const magellan_messages::MsgSerialPortLine::ConstPtr &input_data)
{
    if (input_data->data.size() > 0)
    {
        right_response.reserve(input_data->data.size());
        std::copy(input_data->data.begin(), input_data->data.end(), std::back_inserter(right_response));
        right_response_valid = true;
    }
}

void command_message_received_callback(const magellan_messages::MsgMagellanDrive::ConstPtr &input_data)
{
    drive(input_data->left_throttle, input_data->right_throttle);
}

void sigint_handler(int signal)
{
    drive(0, 0);
    ros::shutdown();
}

void drive(double left_throttle, double right_throttle)
{
    // the signs of the directions are determined experimentally.
    uint8_t motor_fl_command_id = (left_throttle < 0 ? CMD_M1_FORWARD : CMD_M1_BACKWARD);
    uint8_t motor_fl_value = std::abs(left_throttle) / 100.0 * 127.0;
    uint8_t motor_fr_command_id = (right_throttle > 0 ? CMD_M1_FORWARD : CMD_M1_BACKWARD);
    uint8_t motor_fr_value = std::abs(right_throttle) / 100.0 * 127.0;
    uint8_t motor_bl_command_id = (left_throttle > 0 ? CMD_M2_FORWARD : CMD_M2_BACKWARD);
    uint8_t motor_bl_value = std::abs(left_throttle) / 100.0 * 127.0;
    uint8_t motor_br_command_id = (right_throttle < 0 ? CMD_M2_FORWARD : CMD_M2_BACKWARD);
    uint8_t motor_br_value = std::abs(right_throttle) * 100.0 / 127.0;

    size_t num_retries = 3;

    if ( (!drive_motor(LEFT_ROBOCLAW_ADDRESS, motor_fl_command_id, motor_fl_value, num_retries, left_response_valid, left_response, left_motor_controller_publisher))
            ||
         (!drive_motor(RIGHT_ROBOCLAW_ADDRESS, motor_fr_command_id, motor_fr_value, num_retries, right_response_valid, right_response, right_motor_controller_publisher))
            ||
         (!drive_motor(LEFT_ROBOCLAW_ADDRESS, motor_bl_command_id, motor_bl_value, num_retries, left_response_valid, left_response, left_motor_controller_publisher))
            ||
         (!drive_motor(RIGHT_ROBOCLAW_ADDRESS, motor_br_command_id, motor_br_value, num_retries, right_response_valid, right_response, right_motor_controller_publisher)) )
    {
        ROS_FATAL("Could not set motor controller values. Out of panic, attempting to stop and dying.");
        if (left_throttle != 0 || right_throttle != 0)
        {
            drive(0, 0);
        }

        ros::shutdown();
    }
}

bool drive_motor(uint8_t address, uint8_t command_id, uint8_t value, size_t retries, bool &verify_flag, std::vector<uint8_t> &received_data, ros::Publisher &publisher)
{
    for (size_t i = 0; i < retries; i++)
    {
        received_data.clear();
        send_command(address, command_id, &value, 1, verify_flag, publisher);

        if (verify_motor_speed_command_received(verify_flag, received_data, 1000))
        {
            return true;
        }
    }

    ROS_ERROR("Was not able to set motor at %d with command id %d and value %d", address, command_id, value);
    return false;
}

bool verify_motor_speed_command_received(bool &verify_flag, std::vector<uint8_t> &received_data, size_t usec_timeout)
{
    for (size_t i = 0; i < usec_timeout / 10; i++)
    {
        if (verify_flag)
        {
            verify_flag = false;
            return (received_data.size() > 0 && received_data[0] == 0xFF);
        }

        usleep(10);
        ros::spinOnce();
    }
    return false;
}

void send_command(uint8_t address, uint8_t command_id, uint8_t *data, size_t data_len, bool &verify_flag, ros::Publisher &publisher)
{
    magellan_messages::MsgSerialPortLine cmd_line;
    cmd_line.data.reserve(data_len + 4);

    cmd_line.data.push_back(address);
    cmd_line.data.push_back(command_id);
    
    for (size_t i = 0; i < data_len; i++)
    {
        cmd_line.data.push_back(data[i]);
    }

    append_crc(cmd_line.data);

    cmd_line.header.stamp = ros::Time::now();
    verify_flag = false;
    publisher.publish(cmd_line);
    ros::spinOnce();
}

void append_crc(std::vector<uint8_t> &data)
{
  uint16_t crc = 0;

  for (size_t byte = 0; byte < data.size(); byte++)
  {
    crc = crc ^ ((uint16_t) data[byte] << 8);

    for (unsigned char bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = (crc << 1);
      }
    }
  }

  data.push_back(static_cast<uint8_t>(crc >> 8));
  data.push_back(static_cast<uint8_t>(crc & 0xFF));
}

void set_motor_current_limit(uint8_t motor_address, uint8_t write_command_id, uint8_t read_command_id, uint32_t max_amps, bool &verify_flag, std::vector<uint8_t> &received_data, ros::Publisher &publisher)
{
    magellan_messages::MsgSerialPortLine command;
    command.data.reserve(12);
    size_t max_retry_count = 30;
    
    bool set_successful = false;
    for (size_t retry_count = 0; retry_count < max_retry_count; retry_count++)
    {
        if (retry_count % 10 == 0)
        {
            uint32_t write_data = max_amps * 100; // max current is specified in 10mA units. 

            command.data.push_back(motor_address);
            command.data.push_back(write_command_id);
            command.data.push_back(static_cast<uint8_t>(write_data >> 24));
            command.data.push_back(static_cast<uint8_t>((write_data & 0x00FFFFFF) >> 16));
            command.data.push_back(static_cast<uint8_t>((write_data & 0x0000FFFF) >> 8));
            command.data.push_back(static_cast<uint8_t>((write_data & 0x000000FF)));
            command.data.push_back(0);
            command.data.push_back(0);
            command.data.push_back(0);
            command.data.push_back(0);

            append_crc(command.data);

            verify_flag = false;
            received_data.clear();
            publisher.publish(command);
            ros::spinOnce();
        }
        

        if (verify_motor_speed_command_received(verify_flag, received_data, 1000))
        {
            set_successful = true;
            break;
        }
    }

    if (set_successful)
    {
        for (size_t retry_count = 0; retry_count < max_retry_count; retry_count++)
        {
            if (retry_count % 10 == 0)
            {
                command.data.clear();
                command.data.push_back(motor_address);
                command.data.push_back(read_command_id);

                append_crc(command.data);
                verify_flag = false;
                received_data.clear();
                publisher.publish(command);
                ros::spinOnce();
            }
            

            for (size_t i = 0; i < 3; i++)
            {
                if (verify_flag)
                {
                    if (received_data.size() != 10)
                    {
                        continue;
                    }

                    uint32_t read_current_value = 0;
                    for (int j = 0; j < 4; j++)
                    {
                        read_current_value = read_current_value << 8;
                        read_current_value |= received_data[j];
                    }

                    if (read_current_value == max_amps * 100)
                    {
                        return;
                    }
                }

                usleep(1000 * 1000);
                ros::spinOnce();
            }
        }
    }
    
    ROS_FATAL("Could not set motor current limiting. Out of safety, shutting down.");
    ros::shutdown();
}

bool initialize_motor_controllers()
{
    set_motor_current_limit(LEFT_ROBOCLAW_ADDRESS, CMD_SET_M1_MAX_CURRENT, CMD_GET_M1_MAX_CURRENT, MOTOR_CURRENT_LIMIT_AMPS, left_response_valid, left_response, left_motor_controller_publisher);
    set_motor_current_limit(LEFT_ROBOCLAW_ADDRESS, CMD_SET_M2_MAX_CURRENT, CMD_GET_M2_MAX_CURRENT, MOTOR_CURRENT_LIMIT_AMPS, left_response_valid, left_response, left_motor_controller_publisher);
    set_motor_current_limit(RIGHT_ROBOCLAW_ADDRESS, CMD_SET_M1_MAX_CURRENT, CMD_GET_M1_MAX_CURRENT, MOTOR_CURRENT_LIMIT_AMPS, right_response_valid, right_response, right_motor_controller_publisher);
    set_motor_current_limit(RIGHT_ROBOCLAW_ADDRESS, CMD_SET_M2_MAX_CURRENT, CMD_GET_M2_MAX_CURRENT, MOTOR_CURRENT_LIMIT_AMPS, right_response_valid, right_response, right_motor_controller_publisher);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magellan_drive_hardware");
    ros::NodeHandle nh;

    left_motor_controller_publisher = nh.advertise<magellan_messages::MsgSerialPortLine>("output_left_topic", 1000);
    right_motor_controller_publisher = nh.advertise<magellan_messages::MsgSerialPortLine>("output_right_topic", 1000);

    ros::Subscriber left_serial_subscriber = nh.subscribe<magellan_messages::MsgSerialPortLine>("input_left_serial_topic", 1000, left_serial_message_received_callback);
    ros::Subscriber right_serial_subscriber = nh.subscribe<magellan_messages::MsgSerialPortLine>("input_right_serial_topic", 1000, right_serial_message_received_callback);

    // TODO: It appears as if the publishers won't start working unless we sleep for a few seconds.
    // I don't know how long this has to be, but 5 seconds seems to always work.
    // The alternative is that the entire system dies.
    usleep(5000 * 1000);

    signal(SIGINT, sigint_handler);
    initialize_motor_controllers();

    ros::Subscriber command_message_subscriber = nh.subscribe<magellan_messages::MsgMagellanDrive>("input_command_topic", 1000, command_message_received_callback);

    ros::spin();
}
