#include "../include/obstacle_detector.hpp"
#include "magellan_messages/MsgObstacleDetection.h"
#include "magellan_messages/MsgObstacleDetectorConfig.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>

void ObstacleDetector::set_internal_parameters(
        const magellan_messages::MsgObstacleDetectorConfig &parameters)
{
}

bool ObstacleDetector::detect(
        const sensor_msgs::PointCloud2 &stereo_camera_point_cloud,
        const sensor_msgs::PointCloud2 &rplidar_point_cloud,
        magellan_messages::MsgObstacleDetection &obstacle_detection_result)
{
    if ((stereo_camera_point_cloud.width != this->cloud_width)
            || (stereo_camera_point_cloud.height != this->cloud_height))
    {
        ROS_FATAL("Unexpected point cloud size. Received cloud of size (%d, %d), expected cloud of size (%d, %d).",
                stereo_camera_point_cloud.width, stereo_camera_point_cloud.height,
                this->cloud_width, this->cloud_height);
        return false;
    }

    // TODO: living life on the edge :P
    const StereoVisionPoint_t* stereo_cloud = reinterpret_cast<const StereoVisionPoint_t*>(stereo_camera_point_cloud.data.data());

    this->fill_stereo_metadata(stereo_cloud);
    this->floodfill_traversable_area(stereo_cloud);
    this->floodfill_cones(stereo_cloud);
    this->generate_output_message(stereo_cloud, obstacle_detection_result);
    return true;
}

sensor_msgs::PointCloud2 debug_annotate (
        const sensor_msgs::PointCloud2 &stereo_camera_point_cloud)
{

}

void ObstacleDetector::fill_stereo_metadata(const StereoVisionPoint_t* stereo_cloud)
{
    for (int i = 0; i < this->num_points; i++)
    {
        StereoVisionPointMetadata_t &metadata = this->point_metadata[i];
        const StereoVisionPoint_t &point = stereo_cloud[i];

        if (!std::isfinite(point.x)
                || point.confidence < this->point_min_confidence
                || l2(point.x, point.y, point.z) > this->point_max_distance_sq)
        {
            metadata.is_valid = false;
        }
        else
        {
            metadata.is_valid = true;
            
            if (point.nz <= this->cos_normals_untraversable_thresh)
            {
                metadata.traversability = UNSAFE;
            }
            else
            {
                metadata.traversability = UNSET;
            }

            if (metadata.traversability == UNSAFE
                    && is_cone_color(point))
            {
                metadata.cone_id = 0;
            }
            else
            {
                metadata.cone_id = -1;
            }
        }
    }
}

void ObstacleDetector::floodfill_traversable_area(const StereoVisionPoint_t *stereo_cloud)
{
    std::queue<std::pair<int, int>> queue;

    // Start in the bottom center of the image
    int start_min_y = this->cloud_height - this->floodfill_square_start_size;
    int start_max_y = this->cloud_height;
    int start_min_x = (this->cloud_width / 2) - this->floodfill_square_start_size;
    int start_max_x = (this->cloud_width / 2) + this->floodfill_square_start_size;
    
    for (int y = start_min_y; y < start_max_y; y++)
    {
        for (int x = start_min_x; x < start_max_x; x++)
        {
            if (this->point_metadata[idx(y, x)].is_valid 
                    && stereo_cloud[idx(y, x)].nz <= cos_normals_traversable_thresh)
            {
                queue.emplace(std::make_pair(y, x));
            }
        }
    }

    // Examine neighbors
    while (!queue.empty())
    {
        std::pair<int, int> good_point_coords = queue.front();
        queue.pop();

        int y = good_point_coords.first;
        int x = good_point_coords.second;
        int radius = 1;
        const StereoVisionPoint_t good_point = stereo_cloud[idx(y, x)];
        this->point_metadata[idx(y, x)].traversability = SAFE;
        
        for (int dy = -radius; dy <= radius; dy++)
        {
            for (int dx = -radius; dx <= radius; dx++)
            {
                if (dy == 0 && dx == 0)
                {
                    continue;
                }

                // Check that point is inside cloud
                int ny = y + dy;
                int nx = x + dx;
                if (ny < 0 || ny >= this->cloud_height || nx < 0 || nx >= this->cloud_width)
                {
                    continue;
                }

                // Check that we have data for the point and have not already examined it.
                const StereoVisionPoint_t new_point = stereo_cloud[idx(ny, nx)];
                if (!this->point_metadata[idx(ny, nx)].is_valid
                        || this->point_metadata[idx(ny, nx)].traversability != UNSET)
                {
                    continue;
                }

                // Check that the two points are not too far apart.
                // This would indicate a ledge.
                float distance_sq = this->l2(
                        (new_point.x-good_point.x)*(new_point.x-good_point.x),
                        (new_point.y-good_point.y)*(new_point.y-good_point.y),
                        (new_point.z-good_point.z)*(new_point.z-good_point.z));

                if (distance_sq > this->max_floodfill_neighbor_distance_sq)
                {
                    continue;
                }

                // Check that the normals aren't too far apart.
                float norm_dot = (new_point.nx*good_point.nx) + (new_point.ny*good_point.ny) + (new_point.nz*good_point.nz);
                if (norm_dot < this->min_floodfill_norm_dot)
                {
                    continue;
                }

                // We have a good point. Set its status as "good" and add it to the queue.
                this->point_metadata[idx(ny, nx)].traversability = SAFE;
                queue.emplace(std::make_pair(ny, nx));
            }
        }
    }

    // Any points at this point are unreachable from a good sector.
    // Mark them as unsafe.
    for (int i = 0; i < this->num_points; i++)
    {
        if (this->point_metadata[i].traversability == UNSET)
        {
            this->point_metadata[i].traversability = UNSAFE;
        }
    }
}

void ObstacleDetector::floodfill_cones(const StereoVisionPoint_t *stereo_cloud)
{
    uint32_t cone_id_counter = 1;
    for (int i = 0; i < this->num_points; i++)
    {
        if (this->point_metadata[i].cone_id == 0)
        {
            std::unordered_set<int> cone_points;
            int minX = i % this->cloud_width;
            int maxX = minX;
            int minY = i / this->cloud_height; 
            int maxY = minY;

            std::queue<int> points_to_visit;
            points_to_visit.emplace(i);
            cone_points.insert(i);

            while (!points_to_visit.empty())
            {
                int packed = points_to_visit.front();
                points_to_visit.pop();

                int y = packed / this->cloud_width;
                int x = packed % this->cloud_width;
                const StereoVisionPoint_t &this_point = stereo_cloud[packed];

                minX = std::min(minX, x);
                maxX = std::max(maxX, x);
                minY = std::min(minY, y);
                maxY = std::max(maxY, y);

                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dy == 0 && dx == 0)
                        {
                            continue;
                        }

                        int ny = y + dy;
                        int nx = x + dx;
                        int new_packed = idx(ny, nx);
                        const StereoVisionPoint_t &new_point = stereo_cloud[idx(ny, nx)];

                        float distance_sq = this->l2(
                                (new_point.x - this_point.x),
                                (new_point.y - this_point.y),
                                (new_point.z - this_point.z));

                        bool visited = (cone_points.count(new_packed) != 0);

                        if (this->point_metadata[new_packed].cone_id == 0
                                && !visited
                                && distance_sq < this->max_cone_neighbor_distance_sq)
                        {
                            points_to_visit.emplace(new_packed);
                            cone_points.insert(new_packed);
                        }
                    }
                }
            }

            // TODO: should we use xyz points instead of coordinates? Shouldn't matter because it's rectified, but...
            float cone_width = maxX - minX;
            float cone_height = maxY - minY;
            float aspect_ratio = cone_width / std::max(1.0f, cone_height);

            float is_valid_cone_blob = 
                (aspect_ratio <= this->max_cone_aspect_ratio) 
                && (cone_points.size() >= this->min_cone_point_count);

            for (int packed : cone_points)
            {
                if (is_valid_cone_blob)
                {
                    this->point_metadata[packed].cone_id = cone_id_counter;
                }
                else
                {
                    this->point_metadata[packed].cone_id = -1;
                }
            }
            
            if (is_valid_cone_blob)
            {
                cone_id_counter++;
            }
        }
    }

}

void ObstacleDetector::generate_output_message(
        const StereoVisionPoint_t *stereo_cloud,
        magellan_messages::MsgObstacleDetection &obstacle_detection_result)
{

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min();

    for (int i = 0; i < this->num_points; i++)
    {
        if (this->point_metadata[i].is_valid)
        {
            const StereoVisionPoint_t &point = stereo_cloud[i];
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
    }

    int num_squares_wide = static_cast<int>(ceil((max_x - min_x) / this->occupancy_matrix_grid_square_size));
    int num_squares_tall = static_cast<int>(ceil((max_y - min_y) / this->occupancy_matrix_grid_square_size));

    std::vector<std::map<int, int>> counters(num_squares_wide*num_squares_tall);
    for (int i = 0; i < this->num_points; i++)
    {
        const StereoVisionPointMetadata_t &point_metadata = this->point_metadata[i];
        if (!point_metadata.is_valid)
        {
            continue;
        }

        const StereoVisionPoint_t &point = stereo_cloud[i];

        int occ_x = floor((point.x - min_x) / this->occupancy_matrix_grid_square_size);
        int occ_y = floor((point.y - min_y) / this->occupancy_matrix_grid_square_size);

        if (point_metadata.cone_id > 0)
        {
            counters[idx(occ_y, occ_x)][point_metadata.cone_id]++;
        }
        else if (point_metadata.traversability == SAFE)
        {
            counters[idx(occ_y, occ_x)][0]++;
        }
        else if (point_metadata.traversability == UNSAFE)
        {
            counters[idx(occ_y, occ_x)][-1]++;
        }
        else if (point_metadata.traversability == UNSET)
        {
            counters[idx(occ_y, occ_x)][-2]++;
        }
    }

    obstacle_detection_result.matrix.resize(num_squares_tall*num_squares_wide);
    for (int y = 0; y < num_squares_tall; y++)
    {
        for (int x = 0; x < num_squares_wide; x++)
        {
            std::map<int, int> collected_points = counters[idx(y, x)];

            int max_key = 0;
            int max_value = std::numeric_limits<int>::min();

            for (const auto &kvp : collected_points)
            {
                if (kvp.second > max_value)
                {
                    max_key = kvp.first;
                    max_value = kvp.second;
                }
            }

            if (max_value < this->min_occupancy_matrix_num_points)
            {
                obstacle_detection_result.matrix[idx(y,x)] = -3;
            }
            else
            {
                obstacle_detection_result.matrix[idx(y, x)] = max_key;
            }
        }
    }

    obstacle_detection_result.map_metadata.height = num_squares_tall;
    obstacle_detection_result.map_metadata.width = num_squares_wide;
    obstacle_detection_result.map_metadata.resolution = this->occupancy_matrix_grid_square_size;
}

bool ObstacleDetector::is_cone_color(const StereoVisionPoint_t &stereo_point)
{
    HlsColor_t hls_color = this->rgba_to_hls(stereo_point.rgba_color);

    return ((hls_color.h > this->min_cone_hue)
            && (hls_color.h < this->max_cone_hue)
            && (hls_color.l > this->min_cone_luminance)
            && (hls_color.l < this->max_cone_luminance)
            && (hls_color.l + hls_color.s > this->min_cone_ls_sum));
}

inline HlsColor_t ObstacleDetector::rgba_to_hls(uint32_t rgba_color)
{
    HlsColor_t out;

    float r = static_cast<float>((rgba_color & 0xFF000000) >> 24);
    float g = static_cast<float>((rgba_color & 0x00FF0000) >> 16);
    float b = static_cast<float>((rgba_color & 0x0000FF00) >> 8);

    float max;
    float min = static_cast<float>(std::min(std::min(r, b), g)) / 255.0f;
    bool is_r_max = false;
    bool is_g_max = false;

    if (r > b)
    {
        if (r > g)
        {
            max = static_cast<float>(r) / 255.0;
            is_r_max = true;
        }
        else
        {
            max = static_cast<float>(g) / 255.0;
            is_g_max = true;
        }
    }
    else
    {
        if (b > g)
        {
            max = static_cast<float>(b) / 255.0;
        }
        else
        {
            max = static_cast<float>(g) / 255.0;
            is_g_max = true;
        }
    }

    out.l = (max + min) / 2.0f;

    if (out.l > 0.5f)
    {
        out.s = (max - min) / (2.0f - (max + min));
    }
    else
    {
        out.s = (max - min) / (max + min);
    }

    if (is_r_max)
    {
        out.h = 60.0f * (g - b) / (255.0f * (max - min));
    }
    else if (is_g_max)
    {
        out.h = 120.0f + (60.0f * (b - r) / (255.0f * (max - min)));
    }
    else
    {
        out.h = 240.0f + (60.0f * (r - g) / (255.0f * (max - min)));
    }

    out.h = out.h / 2.0f;
    out.l = out.l * 255.0f;
    out.s = out.s * 255.0f;

    return out;
}