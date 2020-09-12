#include "../include/obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector()
{
    magellan_messages::MsgObstacleDetectorConfig default_config;

    float deg_to_rad = M_PI / 180.0f;
    float in_to_m = 0.0254f;

    default_config.point_min_confidence = 50;
    default_config.point_max_distance = 20;
    default_config.normals_traversable_thresh = 10 * deg_to_rad;
    default_config.normals_untraversable_thresh = 50 * deg_to_rad;
    default_config.floodfill_square_start_size = 0.5;
    default_config.max_floodfill_neighbor_distance = 4.0 * in_to_m; 
    default_config.max_floodfill_neighbor_angle = 10 * deg_to_rad;
    default_config.max_cone_neighbor_distance = 5.0 * in_to_m;
    default_config.max_cone_aspect_ratio = 0.75;
    default_config.min_cone_point_count = 8;
    // TODO: do we need 2-pass for cone color?
    default_config.min_cone_hue = 90;
    default_config.max_cone_hue = 110;
    default_config.min_cone_ls_sum = 275;
    default_config.min_cone_luminance = 60;
    default_config.max_cone_luminance = 160;
    default_config.min_occupancy_matrix_num_points = 1;
    default_config.occupancy_matrix_grid_square_size = 3 * in_to_m;
    default_config.min_num_points_for_speck = 4;
    default_config.downsample_leaf_size = 3 * in_to_m;
    default_config.normal_recompute_search_radius = 0.1f;
    default_config.normal_recompute_max_samples = 100;

    this->set_internal_parameters(default_config);

    this->tmp_upsample_cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    this->tmp_upsample_cloud->reserve(this->tmp_upsample_cloud->width*this->tmp_upsample_cloud->height);
    this->downsampled_cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
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
    this->min_num_points_for_speck = parameters.min_num_points_for_speck;
    this->downsample_leaf_size = parameters.downsample_leaf_size;
    this->normal_recompute_search_radius = parameters.normal_recompute_search_radius;
    this->normal_recompute_max_samples = parameters.normal_recompute_max_samples;
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

    this->voxel_downsample(stereo_cloud);
    this->recompute_normals();
    this->build_search_tree();
    this->floodfill_traversable_area();
    this->floodfill_cones();
    this->generate_output_message(obstacle_detection_result);
    return true;
}

sensor_msgs::PointCloud2 ObstacleDetector::debug_annotate(
        const sensor_msgs::PointCloud2 &stereo_camera_point_cloud)
{
    sensor_msgs::PointCloud2 output_cloud(stereo_camera_point_cloud);
    output_cloud.data.resize(this->downsampled_cloud->size());
    output_cloud.width = this->downsampled_cloud->size();
    output_cloud.height = 1;

    StereoVisionPoint_t* ptr = reinterpret_cast<StereoVisionPoint_t*>(output_cloud.data.data());

    for (int i = 0; i < this->downsampled_cloud->size(); i++)
    {
        pcl::PointXYZRGBNormal p = (*this->downsampled_cloud)[i];
        ptr[i].x = p.x;
        ptr[i].y = p.y;
        ptr[i].z = p.z;
        ptr[i].nx = p.normal_x;
        ptr[i].ny = p.normal_y;
        ptr[i].nz = p.normal_z;
        ptr[i].confidence = 100;
        
        // If it's a cone, color by cone id
        bool is_cone_point = false;
        for (size_t ci = 0; ci < this->cone_indexes.size(); ci++)
        {
            if (cone_indexes[ci].count(i) > 0)
            {
                switch (ci)
                {
                    case 0:
                        ptr[i].rgba_color = 0xFFFF00FF;
                        break;
                    case 1:
                        ptr[i].rgba_color = 0xFFFFFF00;
                        break;
                    case 2:
                        ptr[i].rgba_color = 0xFF00A5FF;
                        break;
                    case 3:
                        ptr[i].rgba_color = 0xFF00FFFF;
                        break;
                    default:
                        throw std::runtime_error("Unexpected cone_id value: " + std::to_string(ci) + ".");
                }

                is_cone_point = true;
                break;
            }
        }

        if (!is_cone_point)
        {
            if (traversable_indexes.count(i) > 0)
            {
                ptr[i].rgba_color = 0xFF00FF00;
            }
            else
            {
                ptr[i].rgba_color = 0xFFFF0000;
            }
        }
    }

    return output_cloud;
}

void ObstacleDetector::voxel_downsample(const StereoVisionPoint_t *cloud)
{
    // downsample, retaining only valid points
    this->tmp_upsample_cloud->clear();

    for (size_t i = 0; i < this->num_points; i++) 
    {
        StereoVisionPoint_t point = cloud[i];
        if (point.confidence > this->point_min_confidence
                && (this->l2(point.x, point.y, point.z) < 100))
        {
            (*this->tmp_upsample_cloud).emplace_back(
                point.x,
                point.y,
                point.z,
                static_cast<uint8_t>(((point.rgba_color & 0x00FF0000) >> 16)),
                static_cast<uint8_t>(((point.rgba_color & 0x0000FF00) >> 8)),
                static_cast<uint8_t>(((point.rgba_color & 0x000000FF))),

                // Normals will be filled in later.
                0,
                0,
                0);
        }
    }

    pcl::VoxelGrid<pcl::PointXYZRGBNormal> filter;
    filter.setInputCloud(this->tmp_upsample_cloud);
    filter.setLeafSize(
        this->downsample_leaf_size,
        this->downsample_leaf_size,
        this->downsample_leaf_size);
    //filter.setMinimumPointsNumberPerVoxel(8);
    //filter.setFilterLimits(-10, 10);
    filter.filter(*(this->downsampled_cloud));
}

void ObstacleDetector::recompute_normals()
{
    if (this->downsampled_cloud->size() > this->gpu_buf.size())
    {
        this->gpu_buf.resize(this->downsampled_cloud->size());
    }

    for (size_t i = 0; i < this->downsampled_cloud->size(); i++)
    {
        pcl::PointXYZRGBNormal p = (*this->downsampled_cloud)[i];
        this->gpu_buf[i].x = p.x;
        this->gpu_buf[i].y = p.y;
        this->gpu_buf[i].z = p.z;
    }

    pcl::gpu::DeviceArray<pcl::PointXYZ> downsampled_cloud_gpu;
    downsampled_cloud_gpu.upload(this->gpu_buf.data(), this->downsampled_cloud->size());

    pcl::gpu::Feature::Normals normals;
    pcl::gpu::NormalEstimation normalEstimator;

    normalEstimator.setInputCloud(downsampled_cloud_gpu);
    normalEstimator.setRadiusSearch(
        this->normal_recompute_search_radius,
        this->normal_recompute_max_samples);

    normalEstimator.setViewPoint(
        0,
        0,
        0.5334f); // height of the bot

    normalEstimator.compute(normals);
    normals.download(this->gpu_buf.data());

    for (size_t i = 0; i < this->gpu_buf.size(); i++)
    {
        pcl::PointXYZ p = this->gpu_buf[i];
        
        (*downsampled_cloud)[i].normal_x = p.data[0];
        (*downsampled_cloud)[i].normal_y = p.data[1];
        (*downsampled_cloud)[i].normal_z = p.data[2];
    }
}

void ObstacleDetector::build_search_tree()
{  
    float resolution = 1.0f;

    this->search_tree = pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr(
              new pcl::search::KdTree<pcl::PointXYZRGBNormal>(resolution));
    
    //this->search_tree = pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>::Ptr(
    //            new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>(resolution));
    
    this->search_tree->setInputCloud(this->downsampled_cloud);
}

void ObstacleDetector::floodfill_traversable_area()
{
    std::queue<int> points_to_visit;

    this->traversable_indexes.clear();

    std::vector<int> points_found;
    std::vector<float> points_sq_distances;

    // Start near the base of the bot
    pcl::PointXYZRGBNormal start_point(0, 0, -0.5334f, 0, 0, 0, 0, 0, 0);
    this->search_tree->radiusSearch(
            start_point, 
            this->floodfill_square_start_size, 
            points_found,
            points_sq_distances);

    for (size_t i = 0; i < points_found.size(); i++)
    {
        int idx = points_found[i];
        pcl::PointXYZRGBNormal p = (*this->downsampled_cloud)[idx];
        
        if (p.normal_z >= this->cos_normals_traversable_thresh)
        {
            points_to_visit.emplace(idx);
        }
    }

    // Examine neighbors
    while (!points_to_visit.empty())
    {
        size_t sz = points_to_visit.size();
        int next_visit_idx = points_to_visit.front();
        points_to_visit.pop();

        // TODO: figure out how to tune this paramter
        // Higher = longer runtime, but less chance of getting "walled in"
        float radius = 2.5 * this->downsample_leaf_size;
        pcl::PointXYZRGBNormal good_point = (*this->downsampled_cloud)[next_visit_idx];

        points_found.clear();
        points_sq_distances.clear();

        this->search_tree->radiusSearch(
            good_point, 
            radius, 
            points_found,
            points_sq_distances);

        for (size_t i = 0; i < points_found.size(); i++)
        {
            int test_idx = points_found[i];

            if (traversable_indexes.count(test_idx) == 0)
            {
                pcl::PointXYZRGBNormal new_point = (*this->downsampled_cloud)[test_idx];

                if (new_point.normal_z > this->cos_normals_untraversable_thresh)
                {
                    // Check that the normals aren't too far apart.
                    float norm_dot = 
                        (new_point.normal_x * good_point.normal_x) 
                      + (new_point.normal_y * good_point.normal_y) 
                      + (new_point.normal_z * good_point.normal_z);

                    if (norm_dot >= this->min_floodfill_norm_dot)
                    {
                        traversable_indexes.emplace(test_idx);
                        points_to_visit.emplace(test_idx);
                    }
                }
            }
        }
    }
}

void ObstacleDetector::floodfill_cones()
{
    this->cone_indexes.clear();

    std::unordered_set<int> visited_points;
    std::vector<int> points_found;
    std::vector<float> points_sq_distances;

    float search_radius = 2.5f * this->downsample_leaf_size;

    for (int i = 0; i < this->downsampled_cloud->size(); i++)
    {
        pcl::PointXYZRGBNormal p = (*this->downsampled_cloud)[i];
        if ((p.normal_z <= this->cos_normals_untraversable_thresh)
             && (this->is_cone_color(p)))
        {
            std::unordered_set<int> cone_points;
            float minX = p.x;
            float maxX = minX;
            float minY = p.y;
            float maxY = minY;

            std::queue<int> points_to_visit;
            points_to_visit.emplace(i);
            cone_points.insert(i);
            visited_points.emplace(i);

            while (!points_to_visit.empty())
            {
                int good_idx = points_to_visit.front();
                points_to_visit.pop();

                pcl::PointXYZRGBNormal good_point = (*this->downsampled_cloud)[good_idx];

                minX = std::min(minX, good_point.x);
                maxX = std::max(maxX, good_point.x);
                minY = std::min(minY, good_point.y);
                maxY = std::max(maxY, good_point.y);

                this->search_tree->radiusSearch(
                    good_point, 
                    search_radius, 
                    points_found,
                    points_sq_distances);

                for (size_t i = 0; i < points_found.size(); i++)
                {
                    int next_idx = points_found[i];
                    if (visited_points.count(next_idx) == 0)
                    {
                        visited_points.emplace(next_idx);

                        pcl::PointXYZRGBNormal next_point = 
                            (*this->downsampled_cloud)[next_idx];

                        if ((next_point.normal_z <= this->cos_normals_untraversable_thresh)
                                && (this->is_cone_color(next_point)))
                        {
                            cone_points.insert(next_idx);
                            points_to_visit.emplace(next_idx);
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

            if (is_valid_cone_blob)
            {
                this->cone_indexes.emplace_back(cone_points); 
            }
        }
    }
}

void ObstacleDetector::generate_output_message(
        magellan_messages::MsgMagellanOccupancyGrid &obstacle_detection_result)
{
    // TODO: For some reason, the Y axis needs to be inverted.
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min();

    for (int i = 0; i < this->downsampled_cloud->size(); i++)
    {
        const pcl::PointXYZRGBNormal p = (*this->downsampled_cloud)[i];
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, (p.y * -1));
        max_y = std::max(max_y, (p.y * -1));
    }

    int num_squares_wide = static_cast<int>(ceil((max_y - min_y) / this->occupancy_matrix_grid_square_size));
    int num_squares_tall = static_cast<int>(ceil((max_x - min_x) / this->occupancy_matrix_grid_square_size));

    std::vector<PointAggregatorType_t> counters(num_squares_wide*num_squares_tall);
    for (int i = 0; i < this->downsampled_cloud->size(); i++)
    {
        const pcl::PointXYZRGBNormal point = (*this->downsampled_cloud)[i];

        int occ_x = (point.x - min_x) / this->occupancy_matrix_grid_square_size;
        int occ_y = ((point.y * -1) - min_y) / this->occupancy_matrix_grid_square_size;
        int packed = (occ_x * num_squares_wide) + occ_y;

        // This gives a small chance that two cones overlap in a square.
        // Even if this does happen in practice, it should be rare.

        bool is_cone_point = false;
        for (size_t ci = 0; ci < this->cone_indexes.size(); ci++)
        {
            if (this->cone_indexes[ci].count(i) > 0)
            {
                counters[packed].cone_count++;
                counters[packed].cone_id = ci + 1;
                is_cone_point = true;
                break;
            }
        }

        
        if (!is_cone_point)
        {
            if (this->traversable_indexes.count(i) > 0)
            {
                counters[packed].safe_count++;
            }
            else
            {
                counters[packed].unsafe_count++;
            }
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

    this->remove_specks(
            obstacle_detection_result,
            this->min_num_points_for_speck);
}

void ObstacleDetector::remove_specks(
        magellan_messages::MsgMagellanOccupancyGrid &obstacle_detection_result,
        int min_num_points_for_speck)
{
    std::unordered_set<int> visited;
    std::unordered_set<int> current_speck;
    std::queue<std::pair<int, int>> queue;

    int num_squares_wide = obstacle_detection_result.map_metadata.width;
    int num_squares_tall = obstacle_detection_result.map_metadata.height;

    for (int y = 0; y < num_squares_wide; y++)
    {
        for (int x = 0; x < num_squares_tall; x++)
        {
            int packed = (x*num_squares_wide) + y;

            if ((obstacle_detection_result.matrix[packed] == -1)
                    &&
                visited.count(packed) == 0)
            {
                current_speck.clear();
                queue = std::queue<std::pair<int, int>>();
                queue.emplace(y, x);

                while (!queue.empty())
                {
                    std::pair<int, int> coords = queue.front();
                    int yy = coords.first;
                    int xx = coords.second;
                    current_speck.emplace((xx*num_squares_wide) + yy);
                    queue.pop();

                    if (yy > 0)
                    {
                        int packed_u = (xx*num_squares_wide) + (yy-1);
                        if ((obstacle_detection_result.matrix[packed_u] == -1) 
                                &&
                             (visited.count(packed_u) == 0))
                        {
                            visited.emplace(packed_u);
                            queue.emplace(yy-1, xx);
                        }
                    }

                    if (yy < num_squares_wide - 1)
                    {
                        int packed_d = (xx*num_squares_wide) + (yy+1);
                        if ((obstacle_detection_result.matrix[packed_d] == -1)
                                &&
                            (visited.count(packed_d) == 0))
                        {
                            visited.emplace(packed_d);
                            queue.emplace(yy+1, x);
                        }
                    }

                    if (xx > 0)
                    {
                        int packed_l = ((xx - 1)*(num_squares_wide)) + yy;
                        if ((obstacle_detection_result.matrix[packed_l] == -1)
                                &&
                            (visited.count(packed_l) == 0))
                        {
                            visited.emplace(packed_l);
                            queue.emplace(yy, xx-1);
                        }
                    }

                    if (xx < num_squares_tall - 1)
                    {
                        int packed_r = ((xx + 1) * (num_squares_wide)) + yy;
                        if ((obstacle_detection_result.matrix[packed_r] == -1)
                                &&
                            (visited.count(packed_r) == 0))
                        {
                            visited.emplace(packed_r);
                            queue.emplace(yy, xx+1);
                        }
                    }
                }
            }

            if (current_speck.size() < min_num_points_for_speck)
            {
                for (const int& packed : current_speck)
                {
                    obstacle_detection_result.matrix[packed] = 0;
                }
            }
        }
    }

}

bool ObstacleDetector::is_cone_color(const pcl::PointXYZRGBNormal& point)
{
    HlsColor_t hls_color = this->rgba_to_hls(point.rgba);

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
