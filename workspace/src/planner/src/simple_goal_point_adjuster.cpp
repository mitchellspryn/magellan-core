#include "../include/simple_goal_point_adjuster.hpp"
#include "geometry_msgs/Point.h"
#include "magellan_messages/MsgMagellanOccupancyGrid.h"
#include <cstddef>
#include <unordered_set>
#include <algorithm>

bool SimpleGoalPointAdjuster::adjust_goal_point(
    const GlobalMap& map,
    geometry_msgs::Point& goal_point_estimate)
{
    std::unordered_set<int> visited;
    std::queue<int> traversal_queue;

    const magellan_messages::MsgMagellanOccupancyGrid& grid = map.get_map();
    float map_resolution = map.get_resolution();
    int search_radius = static_cast<int>(
        max_adjust_distance_m / map_resolution);  
    int map_width = static_cast<int>(grid.map_metadata.width);
    int map_height = static_cast<int>(grid.map_metadata.height);

    OccupancyGridSquare_t estimated_square = map.real_to_grid(
        goal_point_estimate);

    int packed_index = (estimated_square.y * map_width) + estimated_square.x;
    visited.emplace(packed_index);
    traversal_queue.push(packed_index);

    while (!traversal_queue.empty())
    {
        packed_index = traversal_queue.front();
        traversal_queue.pop();

        int y = packed_index / map_width;
        int x = packed_index % map_width;

        // Cones have value > 0
        // TODO: For now, assume 1 cone.
        if (grid.matrix[packed_index] > 0) 
        {
            goal_point_estimate = this->find_centroid(
                grid,
                x,
                y);

            return true;
        }

        int min_y = std::max(std::max(0, estimated_square.y-search_radius), y-1);
        int max_y = std::min(std::min(map_height-1, estimated_square.y+search_radius), y+1);
        int min_x = std::max(std::max(0, estimated_square.x+search_radius), x-1);
        int max_x = std::min(std::min(map_width-1, estimated_square.x+search_radius), x+1);

        for (int yy = min_y; yy <= max_y; yy++)
        {
            for (int xx = min_x; xx <= max_x; xx++)
            {
                if ((xx == x) && (yy == y))
                {
                    continue;
                }

                packed_index = (yy * map_width) + xx;
                if (!visited.count(packed_index))
                {
                    visited.emplace(packed_index);
                    traversal_queue.emplace(packed_index);
                }
            }
        }
    }

    return false;
}

geometry_msgs::Point SimpleGoalPointAdjuster::find_centroid(
    const magellan_messages::MsgMagellanOccupancyGrid& grid,
    int startX,
    int startY)
{
    int map_width = static_cast<int>(grid.map_metadata.width);
    int map_height = static_cast<int>(grid.map_metadata.height);

    std::unordered_set<int> visited;
    std::queue<int> traversal_queue;

    int packed_index = (map_width * startY) + startX;
    traversal_queue.push(packed_index);

    double running_avg_x = 0;
    double running_avg_y = 0;
    double num_points = 0;

    while (!traversal_queue.empty())
    {
        packed_index = traversal_queue.front();
        traversal_queue.pop();

        int y = packed_index / map_width;
        int x = packed_index % map_width;

        running_avg_x = (running_avg_x * num_points) + static_cast<double>(x) / (num_points + 1);
        running_avg_y = (running_avg_y * num_points) + static_cast<double>(y) / (num_points + 1);
        num_points++;

        constexpr int radius = 1;
        for (int yy = std::max(0, y-radius); yy <= std::min(map_height-1, y+radius); yy++)
        {
            for (int xx = std::max(0, x-radius); xx <= std::min(map_width-1, x+radius); xx++)
            {
                packed_index = (map_width*startY) + startX;
                if (!visited.count(packed_index)) 
                {
                    visited.emplace(packed_index);
                    traversal_queue.push(packed_index);
                }
            }
        }

    }

    geometry_msgs::Point p;
    p.x = running_avg_x;
    p.y = running_avg_y;
    p.z = 0;

    return p;
}


