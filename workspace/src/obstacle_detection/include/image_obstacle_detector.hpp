#ifndef IMAGE_OBSTACLE_DETECTOR_HPP
#define IMAGE_OBSTACLE_DETECTOR_HPP

#include <algorithm>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "contracts/detecting_sensor.hpp"
#include "contracts/image_obstacle.hpp"
#include "contracts/obstacle.hpp"
#include "contracts/obstacle_type.hpp"
#include "contracts/perception_params.hpp"

namespace magellan
{
    namespace obstacle_detection
    {
        class ImageObstacleDetector
        {
            public:
                ImageObstacleDetector(const PerceptionParams &params);

                bool detect_obstacles(
                        const cv::Mat &input_image_rgb,  
                        const cv::Mat &input_image_depth,
                        std::vector<std::unique_ptr<Obstacle>> &output_obstacles);

                bool set_parameters(const PerceptionParams &params);

            private:
                class CameraBlob
                {
                    public:
                        int min_x;
                        int max_x;
                        int min_y;
                        int max_y;
                        int id;
                        std::vector<cv::Point2i> pixels;
                        std::vector<cv::Point3f> real_world_coordinates;
                };

                PerceptionParams _params;

                int _min_cone_h;
                int _max_cone_h;
                int _min_cone_luminance;
                int _max_cone_luminance;
                int _min_lum_sat_sum;
                float _cone_height_in_mm;
                float _max_aspect_ratio;
                float _min_point_percentage;
                float _depth_estimation_point_percentage;

                static constexpr float _max_depth = 3495.0f;
                static constexpr float _min_depth = 6.0f; // xtion is mounted behind 0.25" plastic
                static constexpr float _xtion_vertical_fov_radians = 0.785398; // 45 degrees
                static constexpr float _xtion_horizontal_fov_radians = 1.01229; // 58 degrees

                void obtain_cone_obstacles_from_image(const cv::Mat &input_image_rgb,
                        const cv::Mat &input_image_depth,
                        std::vector<std::unique_ptr<Obstacle>> &output_obstacles);

                void obtain_other_obstacles_from_depth_image(const cv::Mat &input_image_depth, 
                        std::vector<std::unique_ptr<Obstacle>> &obstacles);

                void get_cones_in_image(const cv::Mat &input_image_rgb, 
                        std::vector<std::unique_ptr<CameraBlob>> &blobs);

                void estimate_cone_3d_point_cloud(const cv::Mat &input_image_depth,
                        std::unique_ptr<CameraBlob> &blob);

                bool parse_perception_parameters(const PerceptionParams &params);

        };
    }
}

#endif
