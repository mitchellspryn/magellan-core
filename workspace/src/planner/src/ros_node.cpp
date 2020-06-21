#include <mutex>
#include <ros/ros.h>
#include <thread>

#include <magellan_messages/MsgMagellanDrive.h>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>

#include "../include/a_star_path_generator.hpp"
#include "../include/geometry_utils.hpp"
#include "../include/global_map.hpp"
#include "../include/last_seen_global_map.hpp"
#include "../include/motor_signal_generator.hpp"
#include "../include/path_generator.hpp"
#include "../include/path_validator.hpp"
#include "../include/pid_motor_signal_generator.hpp"
#include "../include/planner.hpp"
#include "../include/simple_path_validator.hpp"
#include "../include/voting_global_map.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"

static magellan_messages::MsgMagellanOccupancyGrid g_occupancy_grid;
static magellan_messages::MsgZedPose g_zed_pose;
static bool g_occupancy_grid_initialized = false;
static bool g_zed_pose_initialized = false;
static Planner g_planner;

static std::mutex g_planner_mutex;

static ros::Publisher g_signals_publisher;

void occupancy_grid_received_callback(const magellan_messages::MsgMagellanOccupancyGrid::ConstPtr &grid)
{
    g_planner_mutex.lock();
    g_occupancy_grid = *grid;
    g_occupancy_grid_initialized = true;
    g_planner_mutex.unlock();
}

void pose_received_callback(const magellan_messages::MsgZedPose::ConstPtr &pose)
{
    g_planner_mutex.lock();
    g_zed_pose = *pose;
    g_zed_pose_initialized = true;
    g_planner_mutex.unlock();
}

void destination_received_callback(const geometry_msgs::Point::ConstPtr &point)
{
    // TODO: log?
    g_planner_mutex.lock();
    g_planner.set_goal_position(*point);
    g_planner_mutex.unlock();
}

void planner_thread(const ros::TimerEvent &event)
{
    if (!g_occupancy_grid_initialized || !g_zed_pose_initialized)
    {
        return;
    }

    g_planner_mutex.lock();
    magellan_messages::MsgMagellanDrive output = g_planner.run_planner(
        g_zed_pose,
        g_occupancy_grid);
    g_planner_mutex.unlock();

    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    g_signals_publisher.publish(output);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "planner");

    // TODO: command line args?
    
    ros::NodeHandle nh;
    g_signals_publisher = nh.advertise<magellan_messages::MsgMagellanDrive>("output_motor_control_signals", 1000);

    ros::Subscriber pose_subscriber = nh.subscribe<magellan_messages::MsgZedPose>("input_topic_pose", 1, pose_received_callback);
    ros::Subscriber occupancy_grid_subscriber = nh.subscribe<magellan_messages::MsgMagellanOccupancyGrid>("input_topic_occupancy_grid", 1, occupancy_grid_received_callback);
    ros::Subscriber destination_subscriber = nh.subscribe<geometry_msgs::Point>("input_topic_destination", 1, destination_received_callback);

    // Run at 10 hz
    // TODO: make configurable?
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), planner_thread);
    timer.start();

    ros::spin();
}
