#include "../include/last_seen_global_map.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "magellan_messages/MsgMagellanOccupancyGrid.h"
#include <atomic>

LastSeenGlobalMap::LastSeenGlobalMap(
    const geometry_msgs::Pose &goal_pose,
    float x_border,
    float y_border,
    float resolution_in_m)
{
    this->min_world_x = (-1 * x_border) + std::min(0.0, goal_pose.position.x);
    this->max_world_x = x_border + std::max(0.0, goal_pose.position.x);
    this->min_world_y = (-1 * y_border) + std::min(0.0, goal_pose.position.y);
    this->max_world_y = y_border + std::max(0.0, goal_pose.position.y);
    this->resolution_in_m = resolution_in_m;

    this->num_x_squares = static_cast<int>(
            ceil((max_world_x - min_world_x) / resolution_in_m));

    this->num_y_squares = static_cast<int>(
        ceil((max_world_y - min_world_y) / resolution_in_m));

    this->grid.map_metadata.height = this->num_y_squares;
    this->grid.map_metadata.width = this->num_x_squares;
    this->grid.map_metadata.resolution = resolution_in_m;
    this->grid.map_metadata.origin.position.x = this->min_world_x;
    this->grid.map_metadata.origin.position.y = this->min_world_y;

    this->grid.matrix.clear();
    this->grid.matrix.resize(this->num_x_squares*this->num_y_squares, 0);

}

void LastSeenGlobalMap::update_map(
    const magellan_messages::MsgZedPose &pose,
    const magellan_messages::MsgMagellanOccupancyGrid &obstacles)
{
    geometry_msgs::Quaternion inversePoseRotation;
    inversePoseRotation.w = pose.pose.pose.orientation.w;
    inversePoseRotation.x = -pose.pose.pose.orientation.x;
    inversePoseRotation.y = -pose.pose.pose.orientation.y;
    inversePoseRotation.z = -pose.pose.pose.orientation.z;

    geometry_msgs::Point inversePoseTranslation;
    inversePoseTranslation.x = -pose.pose.pose.position.x;
    inversePoseTranslation.y = -pose.pose.pose.position.y;
    inversePoseTranslation.z = -pose.pose.pose.position.z;

    for (size_t i = 0; i < obstacles.matrix.size(); i++)
    {
        if (obstacles.matrix[i] == -1 //obstacle
                ||
            obstacles.matrix[i] > 0) // cone
        {
            geometry_msgs::Point local_point;
            local_point.x = 
                (static_cast<int>(i % obstacles.map_metadata.width) + 0.5) * obstacles.map_metadata.resolution;
            local_point.y = 
                (static_cast<int>(i / obstacles.map_metadata.height) + 0.5) * obstacles.map_metadata.resolution;
            local_point.z = 0;

            // Transform into zed coordinates
            local_point.x += obstacles.map_metadata.origin.position.x;
            local_point.y += obstacles.map_metadata.origin.position.y;
            // z == 0, always
            // Also axis aligned
            
            // Transform to world coordinates
            geometry_msgs::Point world_point = GeometryUtils::RotateByQuaternion(local_point, inversePoseRotation);
            world_point.x += inversePoseTranslation.x;
            world_point.y += inversePoseTranslation.y;
            world_point.z += inversePoseTranslation.z;

            // Bin
            int x_bin = (world_point.x - min_world_x) / resolution_in_m;
            int y_bin = (world_point.y - min_world_y) / resolution_in_m;

            x_bin = std::max(std::min(this->num_x_squares - 1, x_bin), 0);
            y_bin = std::max(std::min(this->num_y_squares - 1, y_bin), 0);

            this->grid.matrix[(y_bin*this->num_x_squares)  + x_bin] = obstacles.matrix[i];
        }
    }
}

const magellan_messages::MsgMagellanOccupancyGrid& LastSeenGlobalMap::get_map() const
{
    return this->grid;
}

OccupancyGridSquare_t LastSeenGlobalMap::real_to_grid(
    const geometry_msgs::Point &real_world_point) const
{
    OccupancyGridSquare_t square;
    
    square.x = static_cast<int>((real_world_point.x - this->min_world_x) / this->resolution_in_m);
    square.y = static_cast<int>((real_world_point.y - this->min_world_y) / this->resolution_in_m);

    square.x = std::max(std::min(this->num_x_squares-1, square.x), 0);
    square.y = std::max(std::min(this->num_y_squares-1, square.y), 0);

    return square;
}

geometry_msgs::Point LastSeenGlobalMap::grid_to_real(
    const OccupancyGridSquare_t &square) const
{
    geometry_msgs::Point point;
    
    point.x = (square.x * this->resolution_in_m) + this->min_world_x;
    point.y = (square.y * this->resolution_in_m) + this->min_world_y;
    point.z = 0;

    return point;
}
