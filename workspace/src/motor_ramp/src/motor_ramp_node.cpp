#include <magellan_messages/MsgMagellanDrive.h>
#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

float left_set_point = 0.0f;
float right_set_point = 0.0f;
float left_current = 0.0f;
float right_current = 0.0f;
float max_increase_per_tick = 0.0f;
float max_decrease_per_tick = 0.0f;

ros::Publisher publisher;

constexpr float ticks_per_second = 10.0f;

float clamp(float value, float lo, float hi)
{
    return std::max(std::min(value, hi), lo);
}

void message_received_callback(const magellan_messages::MsgMagellanDrive::ConstPtr &msg)
{
    left_set_point = msg->left_throttle;
    right_set_point = msg->right_throttle;
}

// TODO: This implements linear interpolation. 
// Should we do something fancier like sinusoidal interpolation?
void timer_callback(const ros::TimerEvent &e)
{
    if (left_set_point != left_current 
            || right_set_point != right_current)
    {
        magellan_messages::MsgMagellanDrive m;
        m.left_throttle = left_current + clamp(left_set_point - left_current, max_decrease_per_tick, max_increase_per_tick);
        m.right_throttle = right_current + clamp(right_set_point - right_current, max_decrease_per_tick, max_increase_per_tick);

        m.header.stamp = ros::Time::now();

        publisher.publish(m);

        left_current = m.left_throttle;
        right_current = m.right_throttle;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_ramp");
    ros::NodeHandle nh;

    int c;
    while((c = getopt(argc, argv, "m:")) != -1)
    {
        switch (c)
        {
            case 'm':
                try
                {
                    float max_change_per_second = std::stof(std::string(optarg));
                    if (max_change_per_second <= 0)
                    {
                        ROS_FATAL("Invalid max_change_per_second: %s. Expected float > 0.", 
                                optarg);
                        return 1;
                    }

                    max_increase_per_tick = max_change_per_second / ticks_per_second;
                    max_decrease_per_tick = -1.0f * max_increase_per_tick;
                }
                catch (std::exception ex)
                {
                    ROS_FATAL("Could not parse command line argument for max_change_per_second: %s. Expected float > 0", optarg);

                    return 1;
                }
                break;
            default:
                ROS_FATAL("Received unrecognized command line argument: %c", c);
                return 1;
        }
    }

    ros::Timer timer = nh.createTimer(ros::Duration(1.0f / ticks_per_second), timer_callback);

    ros::Subscriber subscriber = nh.subscribe<magellan_messages::MsgMagellanDrive>("input_topic", 1, message_received_callback);

    publisher = nh.advertise<magellan_messages::MsgMagellanDrive>("output_topic", 1000);

    ros::spin();
    return 0;
}
