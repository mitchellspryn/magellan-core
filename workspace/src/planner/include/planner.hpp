#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <magellan_messages/MsgMagellanDrive.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <magellan_messages/MsgMagellanPlannerDebug.h>
#include <magellan_messages/MsgZedPose.h>
#include <memory>
#include <ros/ros.h>

#include "a_star_path_generator.hpp"
#include "global_map.hpp"
#include "goal_point_adjuster.hpp"
#include "last_seen_global_map.hpp"
#include "motor_signal_generator.hpp"
#include "path_generator.hpp"
#include "path_validator.hpp"
#include "pid_motor_signal_generator.hpp"
#include "simple_goal_point_adjuster.hpp"
#include "simple_path_validator.hpp"

class Planner
{
    public:
        Planner();

        magellan_messages::MsgMagellanDrive run_planner(
            const magellan_messages::MsgZedPose &pose,
            const magellan_messages::MsgMagellanOccupancyGrid &obstacles,
            magellan_messages::MsgMagellanPlannerDebug &debug_msg);

        void set_goal_position(const geometry_msgs::Point &position);
        const geometry_msgs::Point& get_goal_position();

    private:
        std::unique_ptr<GlobalMap> global_map;
        std::unique_ptr<MotorSignalGenerator> motor_signal_generator;
        std::unique_ptr<PathGenerator> path_generator;
        std::unique_ptr<PathValidator> path_validator;
        std::unique_ptr<GoalPointAdjuster> goal_point_adjuster;

        geometry_msgs::Point goal_position;
        nav_msgs::Path planned_path;

        float goal_tolerance;
        bool reached_goal;

        void reinitialize();
};

#endif
