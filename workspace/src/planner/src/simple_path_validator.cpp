#include "../include/simple_path_validator.hpp"

bool SimplePathValidator::is_path_valid(
    const magellan_messages::MsgZedPose &current_pose,
    const nav_msgs::Path &path,
    const GlobalMap &global_map)
{
    if (path.poses.size() == 0)
    {
        return true;
    }

    if (!this->is_segment_valid(
            current_pose.pose.pose.position,
            path.poses[0].pose.position,
            global_map,
            (path.poses.size() == 1)))
    {
        return false;
    }

    for (int i = 1; i < path.poses.size(); i++)
    {
        if (!this->is_segment_valid(
                path.poses[i-1].pose.position,
                path.poses[i].pose.position,
                global_map,
                (i == path.poses.size() - 1)))
        {
            return false;
        }
    }
    
    return true;
}

bool SimplePathValidator::is_segment_valid(
    const geometry_msgs::Point &start,
    const geometry_msgs::Point &end,
    const GlobalMap &global_map,
    bool allow_cone_intersect)
{
    const magellan_messages::MsgMagellanOccupancyGrid &grid = global_map.get_map();
    double step_size_in_m = grid.map_metadata.resolution;
    double start_to_end_x = end.x - start.x;
    double start_to_end_y = end.y - start.y;

    if (start_to_end_x == 0 && start_to_end_y == 0)
    {
        return true;
    }

    double path_length = sqrt((start_to_end_x*start_to_end_x) + (start_to_end_y*start_to_end_y));
    int num_steps = ceil(path_length / step_size_in_m);
    start_to_end_x /= num_steps;
    start_to_end_y /= num_steps;

    geometry_msgs::Point work_point;
    work_point.x = start.x;
    work_point.y = start.y;
    work_point.z = 0;


    for (int i = 0; i < num_steps; i++)
    {
        OccupancyGridSquare_t square = global_map.real_to_grid(work_point);
        int8_t square_value = grid.matrix[(square.y*grid.map_metadata.width)+square.x];

        if ((square_value < 0) 
            ||
            ((square_value > 0) && (!allow_cone_intersect)))
        {
            return false;
        }

        work_point.x += start_to_end_x;
        work_point.y += start_to_end_y;
    }

    return true;
}
