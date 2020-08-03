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

    int packed_index = (estimated_square.x * map_width) + estimated_square.y;
    visited.emplace(packed_index);
    traversal_queue.push(packed_index);

    while (!traversal_queue.empty())
    {
        packed_index = traversal_queue.front();
        traversal_queue.pop();

        int y = packed_index % map_width;
        int x = packed_index / map_width;

        // Cones have value > 0
        // TODO: For now, assume 1 cone.
        if (grid.matrix[packed_index] > 0) 
        {
            goal_point_estimate = this->find_centroid(
                map,
                x,
                y);

            return true;
        }

        int min_y = std::max(std::max(0, estimated_square.y-search_radius), y-1);
        int max_y = std::min(std::min(map_width-1, estimated_square.y+search_radius), y+1);
        int min_x = std::max(std::max(0, estimated_square.x-search_radius), x-1);
        int max_x = std::min(std::min(map_height-1, estimated_square.x+search_radius), x+1);

        for (int yy = min_y; yy <= max_y; yy++)
        {
            for (int xx = min_x; xx <= max_x; xx++)
            {
                if ((xx == x) && (yy == y))
                {
                    continue;
                }

                packed_index = (xx * map_width) + yy;
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
    const GlobalMap& map,
    int startX,
    int startY)
{
    const magellan_messages::MsgMagellanOccupancyGrid grid = map.get_map();
    int map_width = static_cast<int>(grid.map_metadata.width);
    int map_height = static_cast<int>(grid.map_metadata.height);

    std::unordered_set<int> visited;
    std::queue<int> traversal_queue;

    int packed_index = (map_width * startX) + startY;
    traversal_queue.push(packed_index);

    double running_avg_x = 0;
    double running_avg_y = 0;
    double num_points = 0;

    while (!traversal_queue.empty())
    {
        packed_index = traversal_queue.front();
        traversal_queue.pop();

        int y = packed_index % map_width;
        int x = packed_index / map_width;

        running_avg_x = static_cast<double>(((running_avg_x * num_points) + (x))) / (num_points + 1.0);
        running_avg_y = static_cast<double>(((running_avg_y * num_points) + (y))) / (num_points + 1.0);
        num_points++;

        constexpr int radius = 1;
        for (int yy = std::max(0, y-radius); yy <= std::min(map_width-1, y+radius); yy++)
        {
            for (int xx = std::max(0, x-radius); xx <= std::min(map_height-1, x+radius); xx++)
            {
                packed_index = (map_width*xx) + yy;
                if ((grid.matrix[packed_index] > 0)
                        &&
                    (!visited.count(packed_index)))
                {
                    visited.emplace(packed_index);
                    traversal_queue.push(packed_index);
                }
            }
        }

    }

    OccupancyGridSquare_t grid_square;
    grid_square.x = static_cast<int>(round(running_avg_x));
    grid_square.y = static_cast<int>(round(running_avg_y));

    return map.grid_to_real(grid_square);
}

