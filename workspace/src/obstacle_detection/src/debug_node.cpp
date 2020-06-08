#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ros/ros.h"
#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <stdexcept>
#include <string>

#include <magellan_messages/MsgObstacleDetection.h>
#include "../include/obstacle_detector.hpp"
#include "sensor_msgs/PointCloud.h"

float pcd_string_to_val(const std::string &val)
{
    if (val == "nan" || val == "-nan")
    {       
        return std::numeric_limits<float>::quiet_NaN();
    }

    return std::stof(val);
}

std::string get_pcd_string(float val)
{
    if (std::isfinite(val))
    {
        return std::to_string(val);
    }

    return "nan";
}

sensor_msgs::PointField make_point_field(const std::string &name, const unsigned int offset, const unsigned int datatype)
{
    sensor_msgs::PointField pf;
    pf.count = 1;
    pf.datatype = datatype;
    pf.offset = offset;
    pf.name = name;
    return pf;
}

sensor_msgs::PointCloud2 read_pcd(const std::string& file_name)
{
    sensor_msgs::PointCloud2 cloud;

    cloud.fields.push_back(make_point_field("x", 0, 7)); //Float32
    cloud.fields.push_back(make_point_field("y", 4, 7));
    cloud.fields.push_back(make_point_field("z", 8, 7));
    cloud.fields.push_back(make_point_field("nx", 12, 7));
    cloud.fields.push_back(make_point_field("ny", 16, 7));
    cloud.fields.push_back(make_point_field("nz", 20, 7));
    cloud.fields.push_back(make_point_field("confidence", 24, 7)); 
    cloud.fields.push_back(make_point_field("r", 28, 2)); //Uint8
    cloud.fields.push_back(make_point_field("g", 29, 2)); 
    cloud.fields.push_back(make_point_field("b", 30, 2)); 
    cloud.fields.push_back(make_point_field("a", 31, 2)); 

    cloud.point_step = 32;

    std::ifstream file(file_name);      
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open " + file_name + ".");
    }

    std::string line;
    for (int i = 0; i < 5; i++)
    {
        std::getline(file, line);
    }

    // Parse width and height
    std::getline(file, line); //WIDTH
    int width = std::stoi(line.substr(5));

    std::getline(file, line);
    int height = std::stoi(line.substr(6));

    for (int i = 0; i < 3; i++)
    {
        std::getline(file, line);
    }

    cloud.height = height;
    cloud.width = width;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.header.frame_id = "zed";
    cloud.row_step = cloud.point_step * width;
    cloud.data.resize(cloud.point_step * width * height);

    uint8_t* data_ptr = static_cast<uint8_t*>(cloud.data.data());

    while(getline(file, line))
    {
        std::string token;
        std::stringstream ss;
        float x, y, z, nx, ny, nz, confidence;
        uint32_t rgba;

        ss << line;

        std::getline(ss, token, ' ');
        rgba = std::stoul(token);

        std::getline(ss, token, ' ');
        float dirty_confidence = std::stof(token);
        if (dirty_confidence > 100 || dirty_confidence < 0)
        {
            confidence = 0;
        }
        else
        {
            confidence = static_cast<float>(dirty_confidence);
        }

        std::getline(ss, token, ' ');
        x = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        y = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        z = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        nx = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        ny = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        nz = pcd_string_to_val(token);

        memcpy(data_ptr, &x, 4);
        data_ptr += 4;
        memcpy(data_ptr, &y, 4);
        data_ptr += 4;
        memcpy(data_ptr, &z, 4);
        data_ptr += 4;
        memcpy(data_ptr, &nx, 4);
        data_ptr += 4;
        memcpy(data_ptr, &ny, 4);
        data_ptr += 4;
        memcpy(data_ptr, &nz, 4);
        data_ptr += 4;
        memcpy(data_ptr, &confidence, 4);
        data_ptr += 4;
        memcpy(data_ptr, &rgba, 4);
        data_ptr += 4;
    }

    return cloud;
}

void write_pcd(const sensor_msgs::PointCloud2 &cloud, const std::string &file_name)
{
    std::ofstream output_file(file_name, std::ios::out | std::ios::binary);

    if (!output_file) 
    {
        throw std::runtime_error("Could not open " + file_name + ".");
    }

    output_file << "VERSION .7\n";
    output_file << "FIELDS rgb confidence x y z nx ny nz\n";
    output_file << "SIZE 4 4 4 4 4 4 4 4\n";
    output_file << "TYPE U F F F F F F F\n";
    output_file << "COUNT 1 1 1 1 1 1 1 1\n";
    output_file << "WIDTH " << std::to_string(cloud.width) << "\n";
    output_file << "HEIGHT " << std::to_string(cloud.height) << "\n";
    output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    output_file << "POINTS " << std::to_string(cloud.width * cloud.height) << "\n";
    output_file << "DATA ASCII\n";

    const StereoVisionPoint_t* data = reinterpret_cast<const StereoVisionPoint_t*>(cloud.data.data());

    for (int i = 0; i < cloud.width * cloud.height; i++)
    {
        output_file << data->rgba_color << " ";
        output_file << data->confidence << " ";
        output_file << get_pcd_string(data->x) << " ";
        output_file << get_pcd_string(data->y) << " ";
        output_file << get_pcd_string(data->z) << " ";
        output_file << get_pcd_string(data->nx) << " ";
        output_file << get_pcd_string(data->ny) << " ";
        output_file << get_pcd_string(data->nz) << " ";
        output_file << "\n";

        data++;
    }

    output_file.close();
}

cv::Mat visualize_occupancy_matrix(const magellan_messages::MsgObstacleDetection &detection)
{
    int max_image_dim_px = 1000;

    int width_block_size = (max_image_dim_px - (max_image_dim_px % detection.map_metadata.width)) / detection.map_metadata.width;
    int height_block_size = (max_image_dim_px - (max_image_dim_px % detection.map_metadata.height)) / detection.map_metadata.height;

    int block_size = std::min(width_block_size, height_block_size);

    cv::Mat output(detection.map_metadata.height * block_size,
                    detection.map_metadata.width * block_size,
                    CV_8UC3);

    for (int y = 0; y < detection.map_metadata.height; y++)
    {
        for (int x = 0; x < detection.map_metadata.width; x++)
        {
            cv::Point upper_left((x*block_size), (y*block_size));
            cv::Point lower_right(((x+1)*block_size), ((y+1)*block_size));

            constexpr int wall_width = 2;
            cv::Scalar color;
            switch (detection.matrix[(y * detection.map_metadata.width) + x])
            {
                case -3: // not enough points
                    color = cv::Scalar(0, 0, 0);
                    break;
                case -2: // unset
                    color = cv::Scalar(255, 255, 255);
                    break;
                case -1: // unsafe
                    color = cv::Scalar(0, 0, 255);
                    break;
                case 0: // safe
                    color = cv::Scalar(0, 255, 0);
                    break;
                case 1: // first cone
                    color = cv::Scalar(255, 0, 255);
                    break;
                case 2: // second cone
                    color = cv::Scalar(255, 255, 0);
                    break;
                case 3: // third cone
                    color = cv::Scalar(0, 165, 255);
                    break;
                case 4: // fourth cone
                    color = cv::Scalar(0, 255, 255);
                    break;
                default:
                    throw std::runtime_error("Unrecognized occupancy matrix value: " + std::to_string(detection.matrix[(y * detection.map_metadata.width) + x]));
            }

            cv::rectangle(output, upper_left, lower_right, color, -1); //filled
            cv::rectangle(output, upper_left, lower_right, cv::Scalar(0, 0, 0), wall_width); 
        }
    }

    return output;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_obstacle_detection");

    std::string input_file_path = "";

    int c;
    while((c = getopt(argc, argv, "i:")) != -1)
    {
        switch (c)
        {
            case 'i':
                input_file_path = std::string(optarg);
                break;
            default:
                ROS_ERROR("Unrecognized argument %c", c);
                return 1;
        }
    }

    if (input_file_path.size() == 0)
    {
        throw std::runtime_error("No input file specified. Use -i.");
    }

    ROS_ERROR("Creating obstacle detector...");
    ObstacleDetector detector;

    ROS_ERROR("Reading file from %s...", input_file_path.c_str());
    sensor_msgs::PointCloud2 input_cloud = read_pcd(input_file_path);

    ROS_ERROR("Writing to 'echo.pcd'...");
    write_pcd(input_cloud, "echo.pcd");

    ROS_ERROR("Running detection...");
    sensor_msgs::PointCloud2 dummy_cloud;
    magellan_messages::MsgObstacleDetection detection_result;

    std::chrono::high_resolution_clock clk;
    std::chrono::high_resolution_clock::time_point start_time = clk.now();
    bool detection_successful = detector.detect(input_cloud, dummy_cloud, detection_result);
    std::chrono::high_resolution_clock::time_point end_time = clk.now();
    
    if (!detection_successful)
    {
        ROS_ERROR("Detection was not successful.");
        return 1;
    }

    double microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    ROS_ERROR("Detection successful. Ran in %lf microseconds.", microseconds);

    ROS_ERROR("Generating debug point cloud...");
    start_time = clk.now();
    sensor_msgs::PointCloud2 debug_cloud = detector.debug_annotate(input_cloud);
    end_time = clk.now();
    microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    ROS_ERROR("Generated debug cloud in %lf microseconds.", microseconds);

    ROS_ERROR("Writing to 'debug.pcd'...");
    write_pcd(debug_cloud, "debug.pcd");

    ROS_ERROR("Generating occupancy matrix visualization...");
    cv::Mat occupancy_matrix_visualization = visualize_occupancy_matrix(detection_result);

    ROS_ERROR("Writing to 'matrix.bmp'...");
    cv::imwrite("matrix.bmp", occupancy_matrix_visualization);

    ROS_ERROR("Graceful termination.");
    return 0;
}
