#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>
#include <map>
#include <queue>
#include <ros/ros.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <magellan_messages/MsgObstacleDetectorConfig.h>
#include <magellan_messages/MsgObstacleDetection.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "obstacle_detector_types.hpp"
#include "sensor_msgs/PointCloud.h"

class ObstacleDetector
{
    public:
        void set_internal_parameters(
                const magellan_messages::MsgObstacleDetectorConfig &parameters);

        bool detect(
                const sensor_msgs::PointCloud2 &stereo_camera_point_cloud,
                const sensor_msgs::PointCloud2 &rplidar_point_cloud,
                magellan_messages::MsgObstacleDetection &obstacle_detection_result);

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


        StereoVisionPointMetadata_t point_metadata[cloud_width * cloud_height];

        inline float l2(float x, float y, float z) { return (x*x) + (y*y) + (z*z); }
        inline int idx(int y, int x) { return (cloud_width*y) + x; }

        void fill_stereo_metadata(const StereoVisionPoint_t *stereo_cloud);
        void floodfill_traversable_area(const StereoVisionPoint_t *stereo_cloud);
        void floodfill_cones(const StereoVisionPoint_t *stereo_cloud);
        void generate_output_message(
                const StereoVisionPoint_t *stereo_cloud,
                magellan_messages::MsgObstacleDetection &obstacle_detection_result);
        bool is_cone_color(const StereoVisionPoint_t &stereo_point);
        inline HlsColor_t rgba_to_hls(uint32_t rgba_color);

};

#endif
