#ifndef A_STAR_PATH_GENERATOR_HPP
#define A_STAR_PATH_GENERATOR_HPP

#include <algorithm>
#include <functional>
#include <geometry_msgs/Pose.h>
#include <limits>
#include <memory>
#include <magellan_messages/MsgZedPose.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <utility>
#include <vector>

#include "path_generator.hpp"
#include "planner_types.hpp"
#include "path_validator.hpp"
#include "simple_path_validator.hpp"

class AStarPathGenerator : public PathGenerator
{
    public:
        AStarPathGenerator(float obstacle_expansion_size_m);

        virtual bool update_path(
            const magellan_messages::MsgZedPose& current_pose,
            const geometry_msgs::Pose& goal_pose,
            const GlobalMap& world_grid,
            nav_msgs::Path& path,
            magellan_messages::MsgMagellanOccupancyGrid& debug_grid) override;

    private:
        typedef struct AStarPoint
        {
            double sort_value = std::numeric_limits<double>::max();
            int parent_index = -1;
            double cost_from_start = std::numeric_limits<double>::max();
            bool is_obstacle = false;
        } AStarPoint_t;

        std::vector<AStarPoint_t> grid;
        float obstacle_expansion_size_m;

        static constexpr float waypoint_delete_distance_m = 0.15f * 0.15f; // approx 6 inches

        void initialize_grids(
            const GlobalMap& world_grid, 
            magellan_messages::MsgMagellanOccupancyGrid& debug_grid);

        void expand_obstacle(
                int x, 
                int y, 
                int num_blocks,
                const magellan_messages::MsgMagellanOccupancyGrid& grid,
                magellan_messages::MsgMagellanOccupancyGrid& debug_grid);

        void clear_cone(
                int x, 
                int y, 
                int num_blocks,
                const magellan_messages::MsgMagellanOccupancyGrid& grid,
                magellan_messages::MsgMagellanOccupancyGrid& debug_grid);

        bool run_astar(
            const magellan_messages::MsgZedPose& current_pose,
            const geometry_msgs::Pose& final_pose,
            const GlobalMap& world_grid,
            nav_msgs::Path& path,
            magellan_messages::MsgMagellanOccupancyGrid& debug_grid);

        double astar_heuristic(
            const OccupancyGridSquare_t goal,
            const OccupancyGridSquare_t current);

        bool is_path_valid(
            const magellan_messages::MsgZedPose& current_pose,
            const nav_msgs::Path& path,
            const GlobalMap& global_map,
            magellan_messages::MsgMagellanOccupancyGrid& debug_grid);

        bool is_segment_valid(
            const geometry_msgs::Point& start,
            const geometry_msgs::Point& end,
            const GlobalMap& global_map,
            bool allow_cone_intersect,
            magellan_messages::MsgMagellanOccupancyGrid& debug_grid);

};

#endif
