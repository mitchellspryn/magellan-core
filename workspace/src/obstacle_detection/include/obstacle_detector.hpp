#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <queue>
#include <ros/ros.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <magellan_messages/MsgObstacleDetectorConfig.h>
#include <magellan_messages/MsgMagellanOccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/features/don.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/octree/octree_search.h>

#include "obstacle_detector_types.hpp"
#include "sensor_msgs/PointCloud.h"

class ObstacleDetector
{
    public:
        ObstacleDetector();

        void set_internal_parameters(
                const magellan_messages::MsgObstacleDetectorConfig &parameters);

        bool detect(
                const sensor_msgs::PointCloud2 &stereo_camera_point_cloud,
                const sensor_msgs::PointCloud2 &rplidar_point_cloud,
                magellan_messages::MsgMagellanOccupancyGrid &obstacle_detection_result);

        sensor_msgs::PointCloud2 debug_annotate(
                const sensor_msgs::PointCloud2 &stereo_camera_point_cloud);

    private:
        static constexpr int cloud_height = 720;
        static constexpr int cloud_width = 1280;
        static constexpr int num_points = cloud_height * cloud_width;

        float point_min_confidence;
        float point_max_distance_sq;
        float cos_normals_traversable_thresh;
        float cos_normals_untraversable_thresh;
        float floodfill_square_start_size;
        float max_floodfill_neighbor_distance_sq;
        float min_floodfill_norm_dot;
        float max_cone_neighbor_distance_sq;
        float max_cone_aspect_ratio;
        float min_cone_point_count;
        float min_cone_hue;
        float max_cone_hue;
        float min_cone_ls_sum;
        float min_cone_luminance;
        float max_cone_luminance;
        float min_occupancy_matrix_num_points;
        float occupancy_matrix_grid_square_size;
        int min_num_points_for_speck;
        float downsample_leaf_size;
        float normal_recompute_search_radius;
        int normal_recompute_max_samples;
        float octree_resolution;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_upsample_cloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampled_cloud;
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr search_tree;
        //pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>::Ptr search_tree;
        std::unordered_set<int> traversable_indexes;
        std::vector<std::unordered_set<int>> cone_indexes;
        std::vector<pcl::PointXYZ> gpu_buf;

        inline float l2(float x, float y, float z) { return (x*x) + (y*y) + (z*z); }
        inline int idx(int y, int x) { return (cloud_width*y) + x; }
        
        void voxel_downsample(const StereoVisionPoint_t* cloud);
        void recompute_normals();
        void build_search_tree();
        void floodfill_traversable_area();
        void floodfill_cones();
        void generate_output_message(
                magellan_messages::MsgMagellanOccupancyGrid& obstacle_detection_result);
        void remove_specks(
                magellan_messages::MsgMagellanOccupancyGrid& obstacle_detection_result,
                int min_num_points_for_speck);
        bool is_cone_color(const pcl::PointXYZRGBNormal& point);
        inline HlsColor_t rgba_to_hls(uint32_t rgba_color);

};

#endif
