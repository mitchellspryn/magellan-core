#include "../include/obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector()
{
    magellan_messages::MsgObstacleDetectorConfig default_config;

    float deg_to_rad = M_PI / 180.0f;
    float in_to_m = 0.0254f;

    default_config.point_min_confidence = 50;
    default_config.point_max_distance = 20;
    default_config.normals_traversable_thresh = 10 * deg_to_rad;
    default_config.normals_untraversable_thresh = 30 * deg_to_rad;
    default_config.floodfill_square_start_size = 50;
    default_config.max_floodfill_neighbor_distance = 4.0 * in_to_m; 
    default_config.max_floodfill_neighbor_angle = 10 * deg_to_rad;
    default_config.max_cone_neighbor_distance = 5.0 * in_to_m;
    default_config.max_cone_aspect_ratio = 0.75;
    default_config.min_cone_point_count = 500;
    // TODO: do we need 2-pass for cone color?
    default_config.min_cone_hue = 90;
    default_config.max_cone_hue = 110;
    default_config.min_cone_ls_sum = 275;
    default_config.min_cone_luminance = 60;
    default_config.max_cone_luminance = 160;
    default_config.min_occupancy_matrix_num_points = 10;
    default_config.occupancy_matrix_grid_square_size = 3.0 * in_to_m;

    this->set_internal_parameters(default_config);
}

void ObstacleDetector::set_internal_parameters(
        const magellan_messages::MsgObstacleDetectorConfig &parameters)
{
    this->point_min_confidence = parameters.point_min_confidence;
    this->point_max_distance_sq = parameters.point_max_distance * parameters.point_max_distance;
    this->cos_normals_traversable_thresh = cos(parameters.normals_traversable_thresh);
    this->cos_normals_untraversable_thresh = cos(parameters.normals_untraversable_thresh);
    this->floodfill_square_start_size = parameters.floodfill_square_start_size;
    this->max_floodfill_neighbor_distance_sq = parameters.max_floodfill_neighbor_distance * parameters.max_floodfill_neighbor_distance;
    this->min_floodfill_norm_dot = cos(parameters.max_floodfill_neighbor_angle);
    this->max_cone_neighbor_distance_sq = parameters.max_cone_neighbor_distance * parameters.max_cone_neighbor_distance;
    this->max_cone_aspect_ratio = parameters.max_cone_aspect_ratio;
    this->min_cone_point_count = parameters.min_cone_point_count;
    this->min_cone_hue = parameters.min_cone_hue;
    this->max_cone_hue = parameters.max_cone_hue;
    this->min_cone_ls_sum = parameters.min_cone_ls_sum;
    this->min_cone_luminance = parameters.min_cone_luminance;
    this->max_cone_luminance = parameters.max_cone_luminance;
    this->min_occupancy_matrix_num_points = parameters.min_occupancy_matrix_num_points;
    this->occupancy_matrix_grid_square_size = parameters.occupancy_matrix_grid_square_size;
}

bool ObstacleDetector::detect(
        const sensor_msgs::PointCloud2 &stereo_camera_point_cloud,
        const sensor_msgs::PointCloud2 &rplidar_point_cloud,
        magellan_messages::MsgMagellanOccupancyGrid &obstacle_detection_result)
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

sensor_msgs::PointCloud2 ObstacleDetector::debug_annotate (
        const sensor_msgs::PointCloud2 &stereo_camera_point_cloud)
{
    sensor_msgs::PointCloud2 output_cloud(stereo_camera_point_cloud);

    StereoVisionPoint_t* ptr = reinterpret_cast<StereoVisionPoint_t*>(output_cloud.data.data());

    for (int i = 0; i < this->num_points; i++)
    {
        if (!this->point_metadata[i].is_valid)
        {
            ptr->rgba_color = 0x00000000;
        }
        else
        {
            // If it's a cone, color by cone id
            int cone_id = this->point_metadata[i].cone_id;
            TraversabilityClassification_t traversability = this->point_metadata[i].traversability;

            switch (cone_id)
            {
                case -1:
                    switch (traversability)
                    {
                        case UNSAFE:
                            ptr->rgba_color = 0xFFFF0000;
                            break;
                        case SAFE:
                            ptr->rgba_color = 0xFF00FF00;
                            break;
                        case UNSET:
                            throw std::runtime_error("Should not be unset here.");
                        default:
                            throw std::runtime_error("Unexpected traversability type.");
                    }
                    break;
                case 0:
                    throw std::runtime_error("Should not have cone_id of 0 here.");
                case 1:
                    ptr->rgba_color = 0xFFFF00FF;
                    break;
                case 2:
                    ptr->rgba_color = 0xFFFFFF00;
                    break;
                case 3:
                    ptr->rgba_color = 0xFF00A5FF;
                    break;
                case 4:
                    ptr->rgba_color = 0xFF00FFFF;
                    break;
                default:
                    throw std::runtime_error("Unexpected cone_id value: " + std::to_string(cone_id) + ".");
            }
        }

        ptr++;
    }

    return output_cloud;
}

void ObstacleDetector::fill_stereo_metadata(const StereoVisionPoint_t* stereo_cloud)
{
    for (int i = 0; i < this->num_points; i++)
    {
        StereoVisionPointMetadata_t &metadata = this->point_metadata[i];
        const StereoVisionPoint_t &point = stereo_cloud[i];

        if (!std::isfinite(point.x)
                || point.confidence < this->point_min_confidence)
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
    std::queue<int> queue;

    // Start in the bottom center of the image
    int start_min_y = this->cloud_height - this->floodfill_square_start_size;
    int start_max_y = this->cloud_height;
    int start_min_x = (this->cloud_width / 2) - this->floodfill_square_start_size;
    int start_max_x = (this->cloud_width / 2) + this->floodfill_square_start_size;

    for (int y = start_min_y; y < start_max_y; y++)
    {
        for (int x = start_min_x; x < start_max_x; x++)
        {
            int packed = (y*this->cloud_width) + x;
            if (this->point_metadata[packed].is_valid 
                    && stereo_cloud[packed].nz <= cos_normals_traversable_thresh)
            {
                this->point_metadata[packed].traversability = SAFE;
                queue.emplace(packed);
            }
        }
    }

    // Examine neighbors
    while (!queue.empty())
    {
        int good_point_packed = queue.front();
        queue.pop();

        int y = good_point_packed / this->cloud_width;
        int x = good_point_packed % this->cloud_width;
        // TODO: figure out how to tune this paramter
        // Higher = longer runtime, but less chance of getting "walled in"
        int radius = 3;
        const StereoVisionPoint_t good_point = stereo_cloud[good_point_packed];
        
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
                int packed = (ny*this->cloud_width) + nx;
                if (ny < 0 || ny >= this->cloud_height || nx < 0 || nx >= this->cloud_width)
                {
                    continue;
                }

                // Check that we have data for the point and have not already examined it.
                const StereoVisionPoint_t new_point = stereo_cloud[packed];
                if (!this->point_metadata[packed].is_valid
                        || this->point_metadata[packed].traversability != UNSET)
                {
                    continue;
                }

                // Check that the two points are not too far apart.
                // This would indicate a ledge.
                float distance_sq = ((new_point.x-good_point.x)*(new_point.x-good_point.x)) +
                                    ((new_point.y-good_point.y)*(new_point.y-good_point.y)) +
                                    ((new_point.z-good_point.z)*(new_point.z-good_point.z));
               
                if (distance_sq > this->max_floodfill_neighbor_distance_sq)
                {
                    continue;
                }

                //// Check that the normals aren't too far apart.
                float norm_dot = (new_point.nx*good_point.nx) + (new_point.ny*good_point.ny) + (new_point.nz*good_point.nz);
                if (norm_dot < this->min_floodfill_norm_dot)
                {
                    continue;
                }

                // We have a good point. Set its status as "good" and add it to the queue.
                this->point_metadata[packed].traversability = SAFE;
                queue.emplace(packed);
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

                        if ((nx < 0 )
                                || (nx >= this->cloud_width)
                                || (ny < 0) 
                                || (ny >= this->cloud_height))
                        {
                            continue;
                        }

                        int new_packed = idx(ny, nx);

                        if (!this->point_metadata[new_packed].is_valid)
                        {
                            continue;
                        }

                        const StereoVisionPoint_t &new_point = stereo_cloud[idx(ny, nx)];

                        float distance_sq = ((new_point.x - this_point.x)) +
                                            ((new_point.y - this_point.y)) +
                                            ((new_point.z - this_point.z));

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
        magellan_messages::MsgMagellanOccupancyGrid &obstacle_detection_result)
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

    int num_squares_wide = static_cast<int>(ceil((max_y - min_y) / this->occupancy_matrix_grid_square_size));
    int num_squares_tall = static_cast<int>(ceil((max_x - min_x) / this->occupancy_matrix_grid_square_size));

    std::vector<PointAggregatorType_t> counters(num_squares_wide*num_squares_tall);
    for (int i = 0; i < this->num_points; i++)
    {
        const StereoVisionPointMetadata_t &point_metadata = this->point_metadata[i];
        if (!point_metadata.is_valid)
        {
            continue;
        }

        const StereoVisionPoint_t &point = stereo_cloud[i];

        int occ_x = (point.x - min_x) / this->occupancy_matrix_grid_square_size;
        int occ_y = (point.y - min_y) / this->occupancy_matrix_grid_square_size;
        int packed = (occ_x * num_squares_wide) + occ_y;

        // This gives a small chance that two cones overlap in a square.
        // Even if this does happen in practice, it should be rare.
        if (point_metadata.cone_id > 0)
        {
            counters[packed].cone_count++;
            counters[packed].cone_id = point_metadata.cone_id;
        }
        else if (point_metadata.traversability == SAFE)
        {
            counters[packed].safe_count++;
        }
        else if (point_metadata.traversability == UNSAFE)
        {
            counters[packed].unsafe_count++;
        }
    }

    obstacle_detection_result.matrix.resize(num_squares_tall*num_squares_wide);
    for (int y = 0; y < num_squares_wide; y++)
    {
        for (int x = 0; x < num_squares_tall; x++)
        {
            int packed = (x*num_squares_wide) + y;
            const PointAggregatorType_t &agg = counters[packed];

            if (agg.unsafe_count >= agg.safe_count
                    && agg.unsafe_count >= agg.cone_count)
            {
                if (agg.unsafe_count < this->min_occupancy_matrix_num_points)
                {
                    obstacle_detection_result.matrix[packed] = -3;
                }
                else
                {
                    obstacle_detection_result.matrix[packed] = -1;
                }
            }
            else if (agg.safe_count >= agg.unsafe_count
                        && agg.safe_count >= agg.cone_count)
            {
                if (agg.safe_count < this->min_occupancy_matrix_num_points)
                {
                    obstacle_detection_result.matrix[packed] = -3;
                }
                else
                {
                    obstacle_detection_result.matrix[packed] = 0;
                }
            }
            else
            {
                if (agg.cone_count < this->min_occupancy_matrix_num_points)
                {
                    obstacle_detection_result.matrix[packed] = -3;
                }
                else
                {
                    obstacle_detection_result.matrix[packed] = agg.cone_id;
                }
            }
        }
    }

    obstacle_detection_result.map_metadata.height = num_squares_tall;
    obstacle_detection_result.map_metadata.width = num_squares_wide;
    obstacle_detection_result.map_metadata.resolution = this->occupancy_matrix_grid_square_size;
    obstacle_detection_result.map_metadata.origin.orientation.w = 1;
    obstacle_detection_result.map_metadata.origin.orientation.x = 0;
    obstacle_detection_result.map_metadata.origin.orientation.y = 0;
    obstacle_detection_result.map_metadata.origin.orientation.z = 0;
    obstacle_detection_result.map_metadata.origin.position.x = floor(min_x / this->occupancy_matrix_grid_square_size) * this->occupancy_matrix_grid_square_size;
    obstacle_detection_result.map_metadata.origin.position.y = floor(min_y / this->occupancy_matrix_grid_square_size) * this->occupancy_matrix_grid_square_size;
    obstacle_detection_result.map_metadata.origin.position.z = 0;
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

    float b = static_cast<float>((rgba_color & 0xFF000000) >> 24);
    float g = static_cast<float>((rgba_color & 0x00FF0000) >> 16);
    float r = static_cast<float>((rgba_color & 0x0000FF00) >> 8);

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
