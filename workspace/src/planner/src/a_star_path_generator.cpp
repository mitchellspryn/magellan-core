#include "../include/a_star_path_generator.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "magellan_messages/MsgMagellanOccupancyGrid.h"
#include "magellan_messages/MsgZedPose.h"

AStarPathGenerator::AStarPathGenerator(float obstacle_expansion_size_m)
    : obstacle_expansion_size_m(obstacle_expansion_size_m) 
{
    this->validator = std::unique_ptr<SimplePathValidator>(
            new SimplePathValidator());
}

bool AStarPathGenerator::update_path(
    const magellan_messages::MsgZedPose &current_pose,
    const geometry_msgs::Pose &goal_pose,
    const GlobalMap &world_grid,
    nav_msgs::Path &path) 
{
    this->initialize_grids(world_grid);
    return this->run_astar(current_pose, 
            goal_pose,
            world_grid,
            path);
}

void AStarPathGenerator::initialize_grids(const GlobalMap &world_map)
{
    const magellan_messages::MsgMagellanOccupancyGrid world_grid = world_map.get_map();

    this->grid.clear();
    this->grid.resize(world_grid.matrix.size());
    
    int num_expansion_blocks = static_cast<int>(ceil(
        this->obstacle_expansion_size_m / world_grid.map_metadata.resolution));

    for (int i = 0; i < world_grid.matrix.size(); i++)
    {
        if (world_grid.matrix[i] < 0)
        {
            int x = i % world_grid.map_metadata.width;
            int y = i / world_grid.map_metadata.height;

            this->expand_obstacle(x, y, num_expansion_blocks, world_grid);
        }
    }
}

void AStarPathGenerator::expand_obstacle(
    int x, 
    int y, 
    int num_blocks,
    const magellan_messages::MsgMagellanOccupancyGrid &grid)
{
    int min_x = std::max(0, x - num_blocks);
    int min_y = std::max(0, y - num_blocks);
    int max_x = std::min(static_cast<int>(grid.map_metadata.width - 1), x + num_blocks);
    int max_y = std::min(static_cast<int>(grid.map_metadata.height - 1), y + num_blocks);

    for (int yy = min_y; yy <= max_y; yy++)
    {
        for (int xx = min_x; xx <= max_x; xx++)
        {
            this->grid[(yy*grid.map_metadata.width) + xx].is_obstacle = true; 
        }
    }
}

bool AStarPathGenerator::run_astar(
    const magellan_messages::MsgZedPose &current_pose,
    const geometry_msgs::Pose &final_pose,
    const GlobalMap &world_map,
    nav_msgs::Path &path)
{
    path.poses.clear();
    OccupancyGridSquare_t start_square = world_map.real_to_grid(current_pose.pose.pose.position);
    OccupancyGridSquare_t goal_square = world_map.real_to_grid(final_pose.position);

    const magellan_messages::MsgMagellanOccupancyGrid grid = world_map.get_map();
    int map_width = static_cast<int>(grid.map_metadata.width);
    int map_height = static_cast<int>(grid.map_metadata.height);

    int start_packed_index = (map_width * start_square.y) + start_square.x;
    int goal_packed_index = (map_width * goal_square.y) + goal_square.x;

    auto cmp = [&](int &p1,
                   int &p2)
    {
        return this->grid[p1].sort_value > this->grid[p2].sort_value;
    };

    std::priority_queue<
        int,
        std::vector<int>, 
        decltype(cmp)> queue(cmp);

    queue.push(start_packed_index);
    this->grid[start_packed_index].cost_from_start = 0;

    bool path_found = false;
    int max_path_length = 0;
    while (!queue.empty())
    {
        int current_packed_index = queue.top();
        queue.pop();

        if (current_packed_index == goal_packed_index)
        {
            path_found = true;
            break;
        }

        int y = current_packed_index / map_width;
        int x = current_packed_index % map_width;
        int next_cost_from_start = this->grid[current_packed_index].cost_from_start + 1;

        for (int yy = std::max(0, y-1); yy <= std::min(map_height-1, y+1); yy++)
        {
            for (int xx = std::max(0, x-1); xx <= std::min(map_width-1, x+1); xx++)
            {
                if ((xx == x) && (yy == y)) 
                { 
                    continue; 
                }

                int next_index = (yy * map_width) + xx;
                AStarPoint_t &next = this->grid[next_index];
                if (next.is_obstacle
                    ||
                    (next.cost_from_start < next_cost_from_start))
                {
                    continue;
                }

                next.parent_index = current_packed_index;
                max_path_length = std::max(max_path_length, next_cost_from_start);
                next.sort_value = next_cost_from_start + astar_heuristic(
                    OccupancyGridSquare_t(xx, yy),
                    goal_square);
                next.cost_from_start = next.sort_value; // TODO: is this redundant?

                queue.emplace(next_index);
            }
        }
    }

    if (!path_found)
    {
        return false;
    }

    
    // Get list of points, and reorder to point from current pose -> goal pose
    std::vector<int> parent_indexes;
    parent_indexes.reserve(max_path_length + 10); // add a little bit of a fudge factor
    int current_index = goal_packed_index;
    while (current_index != start_packed_index)
    {
        parent_indexes.push_back(current_index);
        current_index = this->grid[current_index].parent_index;
    }

    std::reverse(parent_indexes.begin(), parent_indexes.end());
    
    // Get rid of extra waypoints by checking to see if each point can be "seen" from the current waypoint.
    // TODO: can we make this more efficient? Is it necessary?
    geometry_msgs::PoseStamped current_waypoint;
    current_waypoint.header.frame_id = "map";
    current_waypoint.pose = current_pose.pose.pose;
    path.poses.push_back(current_waypoint);

    geometry_msgs::Point previous_point = current_pose.pose.pose.position;
    geometry_msgs::Point next_point = current_pose.pose.pose.position;
    for (size_t i = 0; i < parent_indexes.size(); i++)
    {
        int index = parent_indexes[i];
        int next_y = index / map_width;
        int next_x = index % map_width;

        previous_point = next_point;
        next_point = world_map.grid_to_real(
            OccupancyGridSquare_t(next_x, next_y));

        if (!validator->is_segment_valid(
            current_waypoint.pose.position,
            next_point,
            world_map,
            false))
        {
            geometry_msgs::PoseStamped tmp;
            tmp.header.frame_id = "map";
            tmp.pose.position = previous_point;
            path.poses.push_back(tmp);

            current_waypoint.pose.position = previous_point;
        }
    }

    geometry_msgs::Point &last_point = path.poses[path.poses.size() - 1].pose.position;

    if ((last_point.x != final_pose.position.x)
        ||
        (last_point.y != final_pose.position.y)
        ||
        (last_point.z != final_pose.position.z))
    {
        geometry_msgs::PoseStamped tmp;
        tmp.header.frame_id = "map";
        tmp.pose.position = final_pose.position;
        path.poses.push_back(tmp); // TODO: is this redundant?
    }

    return true;
}

// Simple L1 norm
double AStarPathGenerator::astar_heuristic(
    const OccupancyGridSquare_t goal,
    const OccupancyGridSquare_t current)
{
    return std::abs(goal.x - current.x) + std::abs(goal.y - current.y); 
}
