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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sl/Camera.hpp>

typedef struct StereoVisionPoint
{
    float x;
    float y;
    float z;
    float nx;
    float ny;
    float nz;
    float confidence;
    uint32_t rgba_color;
} StereoVisionPoint_t;

sensor_msgs::PointField make_point_field(const std::string &name, const unsigned int offset, const unsigned int datatype)
{
    sensor_msgs::PointField pf;
    pf.count = 1;
    pf.datatype = datatype;
    pf.offset = offset;
    pf.name = name;
    return pf;
}

std::string get_pcd_string(float val)
{
    if (std::isfinite(val))
    {
        return std::to_string(val);
    }

    return "nan";
}

uint32_t pack_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    return  ((static_cast<uint32_t>(a) << 24) 
            |
            (static_cast<uint32_t>(b) << 16)
            |
            (static_cast<uint32_t>(g) << 8)
            |
            (static_cast<uint32_t>(r)));
}

sensor_msgs::PointCloud2 read_point_cloud_from_camera()
{
    const int height = 720;
    const int width = 1280;

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
    cloud.height = height;
    cloud.width = width;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.header.frame_id = "zed";
    cloud.row_step = cloud.point_step * width;
    cloud.data.resize(cloud.point_step * width * height);

    uint8_t* cloud_ptr = cloud.data.data();

    sl::Camera camera; 
    
    sl::InitParameters init_parameters;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_minimum_distance = 0.2;
    init_parameters.depth_maximum_distance = 20;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.depth_stabilization = true;
    init_parameters.enable_image_enhancement = true;
    init_parameters.enable_right_side_measure = false;
    init_parameters.sensors_required = false;

    if (camera.open(init_parameters) != sl::ERROR_CODE::SUCCESS) 
    {
        throw std::runtime_error("Could not open camera.");
    }

    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;

    if (camera.grab(runtime_parameters) != sl::ERROR_CODE::SUCCESS)
    {
        throw std::runtime_error("Could not grab frame.");
    }

    sl::Mat image;
    sl::Mat normals;
    sl::Mat point_cloud;
    sl::Mat confidence;

    camera.retrieveImage(image, sl::VIEW::LEFT);
    camera.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
    camera.retrieveMeasure(confidence, sl::MEASURE::CONFIDENCE);
    camera.retrieveMeasure(normals, sl::MEASURE::NORMALS);

    sl::uchar4* image_data = image.getPtr<sl::uchar4>();
    sl::float4* point_cloud_data = point_cloud.getPtr<sl::float4>();
    sl::float4* normals_data = normals.getPtr<sl::float4>();
    sl::float1* confidence_data = confidence.getPtr<sl::float1>();

    float deg_to_rad = M_PI / 180.0f;
    float unrotation_angle_rad = -32.0f * deg_to_rad; // 32 degrees
    
    float c = cos(unrotation_angle_rad);
    float s = sin(unrotation_angle_rad);

    for (int i = 0; i < width*height; i++)
    {
        float x, y, z, nx, ny, nz, confidence;

        if (std::isfinite(point_cloud_data->x))
        {
            int k = 0;
        }

        x = (c * point_cloud_data->x) - (s * point_cloud_data->z);
        y = point_cloud_data->y;
        z = (s * point_cloud_data->x) + (c * point_cloud_data->z);

        nx = (c * normals_data->x) - (s * normals_data->z);
        ny = normals_data->y;
        nz = (s * normals_data->x) + (c * normals_data->z);

        confidence = 100.0f - *confidence_data;

        memcpy(cloud_ptr, &x, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, &y, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, &z, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, &nx, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, &ny, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, &nz, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, &confidence, 4);
        cloud_ptr += 4;
        memcpy(cloud_ptr, image_data, 4);
        cloud_ptr += 4;
        
        ++point_cloud_data;
        ++normals_data;
        ++confidence_data;
        ++image_data;
    }
        

    if (camera.isOpened())
    {
        camera.close();
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_to_pcd");

    ROS_ERROR("Reading frame from camera...");
    sensor_msgs::PointCloud2 cloud = read_point_cloud_from_camera();

    ROS_ERROR("Writing to out.pcd...");
    write_pcd(cloud, "out.pcd");

    ROS_ERROR("Graceful termination.");
    return 0;
}
