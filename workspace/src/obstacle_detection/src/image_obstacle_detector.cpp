#include "image_obstacle_detector.hpp"
#include "contracts/detecting_sensor.hpp"
#include "contracts/image_obstacle.hpp"
#include "contracts/obstacle_type.hpp"
#include <limits>
#include <locale>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace mo = magellan::obstacle_detection;

mo::ImageObstacleDetector::ImageObstacleDetector(const mo::PerceptionParams &params)
    : _params("blah")
{
}

bool mo::ImageObstacleDetector::detect_obstacles(
        const cv::Mat &input_image_rgb,
        const cv::Mat &input_image_depth,
        std::vector<std::unique_ptr<Obstacle>> &output_obstacles)
{
    this->obtain_cone_obstacles_from_image(input_image_rgb, input_image_depth, output_obstacles);
    this->obtain_other_obstacles_from_depth_image(input_image_depth, output_obstacles);
    return true;
}

void mo::ImageObstacleDetector::obtain_cone_obstacles_from_image(
        const cv::Mat &input_image_rgb,
        const cv::Mat &input_image_depth,
        std::vector<std::unique_ptr<mo::Obstacle>> &obstacles)
{
    std::vector<std::unique_ptr<CameraBlob>> cone_blobs;
    this->get_cones_in_image(input_image_rgb, cone_blobs);

    if (cone_blobs.size() > 0)
    {
        for (size_t i = 0; i < cone_blobs.size(); i++)
        {
            this->estimate_cone_3d_point_cloud(input_image_depth, 
                    cone_blobs[i]);

            // TODO: don't be lazy when assigning the angle.
            cv::Point2f center( (cone_blobs[i]->min_x + cone_blobs[i]->max_x) / 2.0f,
                                (cone_blobs[i]->min_y + cone_blobs[i]->max_y) / 2.0f);
            cv::Size2f size( (cone_blobs[i]->max_x - cone_blobs[i]->min_x),
                             (cone_blobs[i]->max_y - cone_blobs[i]->min_y) );
            float angle = 0;

            std::unique_ptr<mo::ImageObstacle> io(
                    new mo::ImageObstacle(
                        cone_blobs[i]->real_world_coordinates,
                        mo::RGB_CAMERA,
                        mo::CONE,
                        cv::RotatedRect(center, size, angle),
                        cone_blobs[i]->pixels.size()));

            obstacles.push_back(std::move(io));
        }
    }
}

void mo::ImageObstacleDetector::obtain_other_obstacles_from_depth_image(
        const cv::Mat &input_image_depth,
        std::vector<std::unique_ptr<mo::Obstacle>> &obstacles)
{

}

void mo::ImageObstacleDetector::get_cones_in_image(
        const cv::Mat &input_image_rgb, 
        std::vector<std::unique_ptr<CameraBlob>> &camera_blobs)
{
    // TODO: should I use CUDA API?
    // Need to perf test on jetson.
    
    // Convert image to Hue-Luminance-Saturation space
    cv::Mat input_image_hls;
    cv::cvtColor(input_image_rgb, input_image_hls, cv::COLOR_BGR2HLS);

    // Threshold the image using the following heuristics:
    // Image should have hue within a particular range (corresponds to approximately orange)
    // Image should not be too bright or dark. This leads to random hue valuess.
    // We want to allow a wider range of hue for low saturation images. 
    //      This can indicate a lighting problem or shadows.
    //      So, we enforce that (luminance + saturation) > (constant). 
    cv::Mat thresholded(input_image_hls.rows, input_image_hls.cols, CV_8UC1, cv::Scalar(0));
    for (size_t y = 0; y < input_image_hls.rows; y++)
    {
        for (size_t x = 0; x < input_image_hls.cols; x++)
        {
            cv::Vec3b pixel = input_image_hls.at<cv::Vec3b>(y, x);

            if (    (pixel[0] >= this->_min_cone_h)
                &&  (pixel[0] <= this->_max_cone_h)
                &&  (pixel[1] >= this->_min_cone_luminance)
                &&  (pixel[1] <= this->_max_cone_luminance)
                &&  (static_cast<int>(pixel[1]) + static_cast<int>(pixel[2]) >= this->_min_lum_sat_sum) )
            {
                thresholded.at<uchar>(y, x) = 1;
            }

        }
    }

    // Open the image to remove noise.
    // Equivalent to 2x iterations of 5x5
    cv::Mat opening_kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(9, 9),
            cv::Point(-1, -1));

    cv::Mat thresholded_smoothed;
    cv::morphologyEx(
            thresholded, 
            thresholded_smoothed, 
            cv::MORPH_OPEN, 
            opening_kernel);

    // Assign each pixel to a blob using queue BFS.
    std::vector<std::unique_ptr<CameraBlob>> temp_blobs;
    temp_blobs.reserve(thresholded_smoothed.rows * thresholded_smoothed.cols / 1000);
    int blob_index = 2;
    for (int y = 0; y < thresholded_smoothed.rows; y++)
    {
        for (int x = 0; x < thresholded_smoothed.cols; x++)
        {
            if (thresholded_smoothed.at<unsigned char>(y, x) == 1)
            {
                std::unique_ptr<CameraBlob> blob(new CameraBlob());

                blob->min_x = std::numeric_limits<int>::max();
                blob->max_x = std::numeric_limits<int>::min();
                blob->min_y = std::numeric_limits<int>::max();
                blob->max_y = std::numeric_limits<int>::min();
                blob->id = blob_index;

                // Note that cv::Point's constructor is (x, y)
                // but .at<> is (y, x)
                std::queue<cv::Point2i> blob_points;
                blob_points.emplace(x, y);
                while (!blob_points.empty())
                {
                    cv::Point2i this_point = blob_points.front();
                    blob_points.pop();

                    thresholded_smoothed.at<uchar>(y, x) = blob_index;

                    blob->min_x = std::min(this_point.x, blob->min_x);
                    blob->max_x = std::max(this_point.x, blob->max_x);
                    blob->min_y = std::min(this_point.y, blob->min_y);
                    blob->max_y = std::max(this_point.y, blob->max_y);

                    blob->pixels.emplace_back(x, y);

                    thresholded_smoothed.at<unsigned char>(y, x) = static_cast<unsigned char>(blob_index);
                    int radius = 1;
                    int min_iter_y = std::max(this_point.y - radius, 0);
                    int max_iter_y = std::min(this_point.y + radius + 1, thresholded_smoothed.rows);
                    int min_iter_x = std::max(this_point.x - radius, 0);
                    int max_iter_x = std::min(this_point.x + radius + 1, thresholded_smoothed.cols);

                    for (int yy = min_iter_y; yy < max_iter_y; yy++)
                    {
                        for (int xx = min_iter_x; xx < max_iter_x; xx++)
                        {
                            if (thresholded_smoothed.at<unsigned char>(yy, xx) == 1)
                            {
                                blob_points.emplace(x, y);
                            }
                        }
                    }
                }

                temp_blobs.push_back(std::move(blob));

                // TODO: this should never happen. If it does, we just have to change a few datatypes.
                // For now, throw an exception if it does.
                if (blob_index == std::numeric_limits<unsigned char>::max())
                {
                    throw std::runtime_error("Ran out of blob indexes.");
                }

                blob_index++;
            }
        }
    }

    // Filter the obtained blobs to remove noise / misshapen blobs.
    int min_required_points = static_cast<int>(this->_min_point_percentage * thresholded_smoothed.cols * thresholded_smoothed.rows);
    for (size_t i = 0; i < temp_blobs.size(); i++)
    {
        float width = temp_blobs[i]->max_x - temp_blobs[i]->min_x;
        float height = temp_blobs[i]->max_y - temp_blobs[i]->min_y;
        float aspect_ratio = width / height;
        
        if (    (aspect_ratio <= this->_max_aspect_ratio)
            &&  (temp_blobs[i]->pixels.size() >= min_required_points))
        {
            camera_blobs.push_back(std::move(temp_blobs[i]));
        }
    }
}

void mo::ImageObstacleDetector::estimate_cone_3d_point_cloud(
        const cv::Mat &input_image_depth,
        std::unique_ptr<CameraBlob> &blob)
{
    int out_of_range_count = 0; 

    for (size_t i = 0; i < blob->pixels.size(); i++)
    {
        float pixel_mm = input_image_depth.at<float>(blob->pixels[i].y, blob->pixels[i].x);
        if (pixel_mm < this->_min_depth || pixel_mm > this->_max_depth)
        {
            out_of_range_count++;
        }
    }

    float estimated_z = -1;
    float middle_height_pixel = input_image_depth.rows / 2.0f;
    float middle_width_pixel = input_image_depth.cols / 2.0f;
    float xtion_half_vertical_fov = this->_xtion_vertical_fov_radians / 2.0f;
    float xtion_half_horizontal_fov = this->_xtion_horizontal_fov_radians / 2.0f;

    // If too many out-of-range pixels, estimate using trig
    if (out_of_range_count > this->_depth_estimation_point_percentage * blob->pixels.size())
    {
        float dist_to_bottom = blob->min_y - middle_height_pixel;
        float dist_to_top = blob->max_y - middle_height_pixel;
        float angle_top = xtion_half_vertical_fov * dist_to_top;
        float angle_bottom = xtion_half_vertical_fov * dist_to_bottom;

        estimated_z = this->_cone_height_in_mm / (tan(angle_top) - tan(angle_bottom));
    }

    for (size_t i = 0; i < blob->pixels.size(); i++)
    {
        float pixel_z = -1;
        if (estimated_z != -1)
        {
            pixel_z = estimated_z;
        }
        else
        {
            pixel_z = input_image_depth.at<float>(blob->pixels[i].y, blob->pixels[i].x);
            pixel_z = std::max(
                        std::min(
                            float(mo::ImageObstacleDetector::_max_depth), pixel_z)
                        , float(mo::ImageObstacleDetector::_min_depth));
        }

        float horizontal_pixel_offset = blob->pixels[i].x - middle_width_pixel;
        float horizontal_pixel_angle = xtion_half_horizontal_fov * horizontal_pixel_offset;
        float pixel_x = pixel_z * tan(horizontal_pixel_angle);

        float vertical_pixel_offset = blob->pixels[i].y - middle_height_pixel;
        float vertical_pixel_angle = xtion_half_vertical_fov * vertical_pixel_offset;
        float pixel_y = pixel_z * tan(vertical_pixel_angle);

        blob->real_world_coordinates.emplace_back(pixel_x, pixel_y, pixel_z);
    }
}


bool mo::ImageObstacleDetector::set_parameters(const PerceptionParams &params)
{
    return true;
}
