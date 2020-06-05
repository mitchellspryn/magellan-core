#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sl/Camera.hpp>

constexpr float DEG_TO_RAD = M_PI / 180.0f;
constexpr float IN_TO_M = 0.0254f;

typedef struct RgbColor
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
} RgbColor_t;

typedef struct HlsColor
{
    float h;
    float l;
    float s;
} HlsColor_t;

typedef enum PointClassification
{
    SAFE = 0,
    BORDERLINE,
    UNSAFE,
    CONE,
    UNSET
} PointClassification_t;

typedef enum ColorBy
{
    RGBA = 0,
    NORM_THRESH,
    SAFE_TRAVERSAL,
    IS_CONE,
    SAFE_TRAVERSAL_WITH_CONE,
} ColorBy_t;

typedef struct PcPoint
{
    float x;
    float y;
    float z;
    float nx;
    float ny;
    float nz;
    uint32_t confidence;
    uint32_t rgba;
    uint32_t confidence_color;
    PointClassification_t normal_threshold_id; 
    PointClassification_t safe_traversal_id;
    bool is_cone_point;
    uint32_t cone_id;
} PcPoint_t;

cv::Mat slMat2cvMat(sl::Mat& input) 
{
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) 
    {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

bool is_valid_point(const PcPoint &point)
{
    return std::isfinite(point.x);
}

uint32_t pack_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    return  ((static_cast<uint32_t>(a) << 24) 
            |
            (static_cast<uint32_t>(r) << 16)
            |
            (static_cast<uint32_t>(g) << 8)
            |
            (static_cast<uint32_t>(b)));
}

RgbColor_t unpack_rgba(uint32_t packed)
{
    RgbColor_t out;

    out.a = static_cast<uint8_t>(((packed & 0xFF000000) >> 24));
    out.r = static_cast<uint8_t>(((packed & 0x00FF0000) >> 16));
    out.g = static_cast<uint8_t>(((packed & 0x0000FF00) >> 8));
    out.b = static_cast<uint8_t>(((packed & 0x000000FF)));

    return out;
}

HlsColor_t rgba_to_hls(RgbColor_t color)
{
    HlsColor_t out;

    float max;
    float min = static_cast<float>(std::min(std::min(color.r, color.b), color.g)) / 255.0f;
    bool is_r_max = false;
    bool is_g_max = false;

    if (color.r > color.b)
    {
        if (color.r > color.g)
        {
            max = static_cast<float>(color.r) / 255.0;
            is_r_max = true;
        }
        else
        {
            max = static_cast<float>(color.g) / 255.0;
            is_g_max = true;
        }
    }
    else
    {
        if (color.b > color.g)
        {
            max = static_cast<float>(color.b) / 255.0;
        }
        else
        {
            max = static_cast<float>(color.g) / 255.0;
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
        out.h = 60.0f * (color.g - color.b) / (255.0f * (max - min));
    }
    else if (is_g_max)
    {
        out.h = 120.0f + (60.0f * (color.b - color.r) / (255.0f * (max - min)));
    }
    else
    {
        out.h = 240.0f + (60.0f * (color.r - color.g) / (255.0f * (max - min)));
    }

    out.h = out.h / 2.0f;
    out.l = out.l * 255.0f;
    out.s = out.s * 255.0f;

    return out;
}

bool is_cone_color(uint32_t point_color)
{
    RgbColor_t rgb_color = unpack_rgba(point_color);
    HlsColor_t hls_color = rgba_to_hls(rgb_color);
    hls_color.h = fmod((hls_color.h + 90.0f), 180.0f);

    if (hls_color.h > 95 && hls_color.h < 105)
    {
        float sum = hls_color.l + hls_color.s;
        if (sum > 275)
        {
            return (hls_color.l < 160 && hls_color.l > 60);
        }
    }

    return false;
}

std::string get_pcd_string(float val)
{
    if (std::isfinite(val))
    {
        return std::to_string(val);
    }

    return "nan";
}

std::string get_pcd_string(uint8_t val)
{
    if (std::isfinite(val))
    {
        return std::to_string(val);
    }

    return "nan";
}

float pcd_string_to_val(const std::string val)
{
    if (val == "nan" || val == "-nan")
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    return std::stof(val);
}

inline uint32_t get_classification_color(const PointClassification_t &thresh)
{
    switch (thresh) 
    {
        case SAFE:
            return pack_rgba(0, 255, 0, 255);
        case UNSAFE:
            return pack_rgba(255, 0, 0, 255);
        case BORDERLINE:
            return pack_rgba(0, 0, 255, 255);
        case CONE:
            return pack_rgba(255, 0, 255, 255);
        case UNSET:
            throw std::runtime_error("Found unset point classification color, not set.");
        default:
            throw std::runtime_error("Unrecognized normal threshold in get_color_normal_thresh: " + std::to_string(thresh) + ".");
    }
}

inline uint32_t get_classification_cone_color(uint32_t cone_id)
{
    switch (cone_id)
    {
        case 1:
            return pack_rgba(255, 0, 255, 255);
        case 2:
            return pack_rgba(0, 255, 255, 255);
        case 3: 
            return pack_rgba(255, 165, 0, 255);
        case 4:
            return pack_rgba(255, 255, 0, 255);
        default:
            throw std::runtime_error("Unexpected color in get_classification_cone_color: " + std::to_string(cone_id));
    }
}

void write_to_pcd(const std::vector<std::vector<PcPoint_t>> &point_cloud, const std::string& file_name, const ColorBy_t colorBy = RGBA)
{
    std::ofstream output_file(file_name, std::ios::out | std::ios::binary);

    if (!output_file) 
    {
        throw std::runtime_error("Could not open " + file_name + ".");
    }

    output_file << "VERSION .7\n";
    output_file << "FIELDS rgb confidence x y z nx ny nz\n";
    output_file << "SIZE 4 4 4 4 4 4 4 4\n";
    output_file << "TYPE U U F F F F F F\n";
    output_file << "COUNT 1 1 1 1 1 1 1 1\n";
    output_file << "WIDTH " << std::to_string(point_cloud[0].size()) << "\n";
    output_file << "HEIGHT " << std::to_string(point_cloud.size()) << "\n";
    output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    output_file << "POINTS " << std::to_string(point_cloud.size() * point_cloud[0].size()) << "\n";
    output_file << "DATA ASCII\n";

    for (size_t y = 0; y < point_cloud.size(); y++)
    {
        for (size_t x = 0; x < point_cloud[y].size(); x++)
        {
            PcPoint_t p = point_cloud[y][x];
            
            switch (colorBy)
            {
                case RGBA:
                    output_file << p.rgba << " ";
                    break;
                case NORM_THRESH:
                    output_file << get_classification_color(p.normal_threshold_id) << " ";
                    break;
                case SAFE_TRAVERSAL:
                    output_file << get_classification_color(p.safe_traversal_id) << " ";
                    break;
                case IS_CONE:
                    if (p.is_cone_point)
                    {
                        output_file << pack_rgba(0, 255, 0, 255) << " ";
                    }
                    else
                    {
                        output_file << pack_rgba(0, 0, 255, 255) << " ";
                    }
                    break;
                case SAFE_TRAVERSAL_WITH_CONE:
                    if (p.is_cone_point)
                    {
                        output_file << get_classification_cone_color(p.cone_id) << " ";
                    }
                    else
                    {
                        output_file << get_classification_color(p.safe_traversal_id) << " ";
                    }

                    break;
                default:
                    throw std::runtime_error("Unrecognized colorBy: " + std::to_string(colorBy) + ".");
            }

            output_file << p.confidence << " ";
            output_file << get_pcd_string(p.x) << " ";
            output_file << get_pcd_string(p.y) << " ";
            output_file << get_pcd_string(p.z) << " ";
            output_file << get_pcd_string(p.nx) << " ";
            output_file << get_pcd_string(p.ny) << " ";
            output_file << get_pcd_string(p.nz) << " ";
            output_file << "\n";
        }
    }

    output_file.close();
}

void remove_point(PcPoint_t &point)
{
    point.x = std::numeric_limits<float>::quiet_NaN();
    point.y = std::numeric_limits<float>::quiet_NaN();
    point.z = std::numeric_limits<float>::quiet_NaN();
    point.nx = std::numeric_limits<float>::quiet_NaN();
    point.ny = std::numeric_limits<float>::quiet_NaN();
    point.nz = std::numeric_limits<float>::quiet_NaN();
}

void remove_points_below_confidence(std::vector<std::vector<PcPoint_t>> &points, uint32_t min_confidence)
{
    for (size_t y = 0; y < points.size(); y++)
    {
        for (size_t x = 0; x < points[y].size(); x++)
        {
            if (points[y][x].confidence <= min_confidence)
            {
                remove_point(points[y][x]);
            }
        }
    }
}

std::vector<std::vector<PcPoint_t>> opencv_point_cloud_from_camera()
{
    const int height = 720;
    const int width = 1280;

    std::vector<std::vector<PcPoint_t>> result;
    result.reserve(height);

    sl::Camera camera; 
    
    sl::InitParameters init_parameters;
    //init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_minimum_distance = 0.2;
    init_parameters.depth_maximum_distance = 20;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.depth_stabilization = true;
    init_parameters.enable_image_enhancement = true;
    init_parameters.enable_right_side_measure = true;
    init_parameters.sensors_required = false;

    if (camera.open(init_parameters) != sl::ERROR_CODE::SUCCESS) 
    {
        throw std::runtime_error("Could not open camera.");
    }

    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;
    //runtime_parameters.sensing_mode = sl::SENSING_MODE::FILL;

    if (camera.grab(runtime_parameters) != sl::ERROR_CODE::SUCCESS)
    {
        throw std::runtime_error("Could not grab frame.");
    }

    sl::Mat left_image;
    sl::Mat right_image;

    camera.retrieveImage(left_image, sl::VIEW::LEFT);
    camera.retrieveImage(right_image, sl::VIEW::RIGHT);

    cv::Mat left_mat = slMat2cvMat(left_image);
    cv::Mat right_mat = slMat2cvMat(right_image);

    cv::Mat left_color = left_mat.clone();

    std::cout << "color rows: " << left_color.rows << std::endl;
    std::cout << "color cols: " << left_color.cols << std::endl;
    std::cout << "color channels: " << left_color.channels() << std::endl;

    double distance_scale = 1000.0;

    cv::Mat left_camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
    left_camera_matrix.at<double>(0,0) = 528.42;
    left_camera_matrix.at<double>(1,1) = 527.91;
    left_camera_matrix.at<double>(2,2) = 1;
    left_camera_matrix.at<double>(0,2) = 671.98;
    left_camera_matrix.at<double>(1,2) = 339.34;

    cv::Mat left_dist_coef = cv::Mat::zeros(5, 1, CV_64FC1);
    left_dist_coef.at<double>(0,0) = -0.0371;
    left_dist_coef.at<double>(1,0) = 0.0051;
    left_dist_coef.at<double>(2,0) = 0.0002;
    left_dist_coef.at<double>(3,0) = 0.0006;
    left_dist_coef.at<double>(4,0) = -0.0032;

    cv::Mat right_camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
    right_camera_matrix.at<double>(0,0) = 528.185;
    right_camera_matrix.at<double>(1,1) = 527.84;
    right_camera_matrix.at<double>(2,2) = 1;
    right_camera_matrix.at<double>(0,2) = 642.3250;
    right_camera_matrix.at<double>(1,2) = 371.1695;

    cv::Mat right_dist_coef = cv::Mat::zeros(5, 1, CV_64FC1);
    right_dist_coef.at<double>(0,0) = -0.0433;
    right_dist_coef.at<double>(1,0) = 0.0130;
    right_dist_coef.at<double>(2,0) = 0.0002;
    right_dist_coef.at<double>(3,0) = 0.0006;
    right_dist_coef.at<double>(4,0) = -0.0032;

    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64FC1); //TODO: does it matter?
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64FC1);
    translation.at<double>(0,0) = 120.1660 / distance_scale;
    translation.at<double>(1,0) = 0.2054 / distance_scale;
    translation.at<double>(2,0) = 0.1285 / distance_scale;

    cv::Mat R1 = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat R2 = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64FC1);
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64FC1);
    cv::Mat Q  = cv::Mat::zeros(4, 4, CV_64FC1);

    cv::Rect roi1, roi2;
    cv::stereoRectify(left_camera_matrix, left_dist_coef, right_camera_matrix, right_dist_coef, cv::Size(width, height), rotation, translation, R1, R2, P1, P2, Q,
            cv::CALIB_ZERO_DISPARITY, -1, cv::Size(width, height), &roi1, &roi2);

    std::cout << "roi1" << std::endl;
    std::cout << roi1 << std::endl;

    std::cout << "Q:" << std::endl;
    std::cout << Q << std::endl;

    cv::Mat disparity;

    int numberOfDisparities = ((width/8) + 15) & -16;

    std::cout << "Number of disparities: " << numberOfDisparities << std::endl;

    //cv::Mat left_gray;
    //cv::Mat right_gray;
    //cv::cvtColor(left_mat, left_gray, CV_BGR2GRAY);
    //cv::cvtColor(right_mat, right_gray, CV_BGR2GRAY);

    //auto bm = cv::StereoBM::create(numberOfDisparities,1); 
    //bm->setROI1(roi1);
    //bm->setROI2(roi2);
    //bm->setPreFilterCap(31);
    //bm->setBlockSize(9);
    //bm->setMinDisparity(0);
    //bm->setNumDisparities(numberOfDisparities);
    //bm->setTextureThreshold(10);
    //bm->setUniquenessRatio(15);
    //bm->setSpeckleWindowSize(100);
    //bm->setSpeckleRange(32);
    //bm->setDisp12MaxDiff(1);

    //bm->compute(left_gray, right_gray, disparity);
    
    auto sbgm = cv::StereoSGBM::create(0, 16, 3);
    sbgm->setPreFilterCap(3);
    int sbgmWinSize = 1;
    sbgm->setBlockSize(sbgmWinSize);

    int channelCount = left_mat.channels();
    //sbgm->setP1(8*channelCount*sbgmWinSize*sbgmWinSize);
    //sbgm->setP2(32*channelCount*sbgmWinSize*sbgmWinSize);
    sbgm->setMinDisparity(0);
    sbgm->setNumDisparities(numberOfDisparities);
    sbgm->setUniquenessRatio(10);
    //sbgm->setSpeckleWindowSize(100);
    //sbgm->setSpeckleRange(32);
    //sbgm->setDisp12MaxDiff(-1);
    sbgm->setMode(cv::StereoSGBM::MODE_HH);

    sbgm->compute(left_mat, right_mat, disparity);

    cv::Mat disp8;
    disparity.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));

    cv::namedWindow("Disparity", cv::WINDOW_AUTOSIZE);
    cv::imshow("Disparity", disp8);
    cv::waitKey(0);

    std::cout << "Disparity shape: (" << disparity.rows << ", " << disparity.cols << ", " << disparity.channels() << ")" << std::endl;
    double min, max;
    cv::minMaxLoc(disparity, &min, &max);
    std::cout << "Disparity min: " << min << std::endl;
    std::cout << "Disparity max: " << max << std::endl;

    cv::Mat points;
    cv::Mat disparityf;
    disparity.convertTo(disparityf, CV_32F, 1.0f/ 16.0f);
    cv::reprojectImageTo3D(disparityf, points, Q, /*handleMissingValues=*/true);

    std::vector<std::vector<PcPoint_t>> output;
    output.reserve(height);
    
    for (int y = 0; y < height; y++)
    {
        std::vector<PcPoint_t> row;
        row.reserve(width);

        for (int x = 0; x < width; x++)
        {
            PcPoint_t point;

            cv::Vec3f p = points.at<cv::Vec3f>(y, x);
            point.x = p.val[0];
            point.y = p.val[1];
            point.z = p.val[2];
            point.nx = -1;
            point.ny = -1;
            point.nz = -1;

            point.confidence = 100;
            cv::Vec4b color = left_color.at<cv::Vec4b>(y, x);
            point.rgba = pack_rgba(color.val[2], color.val[1], color.val[0], 255);

            double max_z = 1.0e4;
            if (fabs(point.z - max_z) < FLT_EPSILON || fabs(point.z) > max_z)
            {
                remove_point(point);
            }

            row.push_back(point);
        }

        output.push_back(row);
    }

    return output;
}

std::vector<std::vector<PcPoint_t>> read_point_cloud_from_camera()
{
    const int height = 720;
    const int width = 1280;

    std::vector<std::vector<PcPoint_t>> result;
    result.reserve(height);

    sl::Camera camera; 
    
    sl::InitParameters init_parameters;
    //init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
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
    //runtime_parameters.sensing_mode = sl::SENSING_MODE::FILL;

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

    for (int y = 0; y < height; y++)
    {
        std::vector<PcPoint_t> row;
        row.reserve(width);
        for (int x = 0; x < width; x++)
        {
            PcPoint_t point;

            point.x = point_cloud_data->x;
            point.y = point_cloud_data->y;
            point.z = point_cloud_data->z;
            point.nx = normals_data->x;
            point.ny = normals_data->y;
            point.nz = normals_data->z;
            point.confidence = 100 - static_cast<uint32_t>(*confidence_data);
            point.rgba = pack_rgba(image_data->b, image_data->g, image_data->r, 255);

            point.normal_threshold_id = UNSET;
            point.safe_traversal_id = UNSET;
            point.cone_id = 0;
            point.is_cone_point = false;
           
            row.push_back(point);

            ++point_cloud_data;
            ++normals_data;
            ++confidence_data;
            ++image_data;
        }
        
        result.push_back(row);
    }

    if (camera.isOpened())
    {
        camera.close();
    }

    return result;
}

std::vector<std::vector<PcPoint_t>> read_point_cloud_from_file(const std::string& file_name)
{
    std::vector<std::vector<PcPoint_t>> result;

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

    std::vector<PcPoint_t> row;
    row.reserve(width);
    result.reserve(height);
    while (getline(file, line))
    {
        std::string token;
        std::stringstream ss;
        PcPoint_t point;
        ss << line;

        std::getline(ss, token, ' ');
        point.rgba = std::stoul(token);

        std::getline(ss, token, ' ');
        long long dirty_confidence = std::stoll(token);
        if (dirty_confidence > 100 || dirty_confidence < 0)
        {
            point.confidence = 0;
        }
        else
        {
            point.confidence = static_cast<uint32_t>(dirty_confidence);
        }

        std::getline(ss, token, ' ');
        point.x = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        point.y = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        point.z = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        point.nx = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        point.ny = pcd_string_to_val(token);

        std::getline(ss, token, ' ');
        point.nz = pcd_string_to_val(token);

        point.normal_threshold_id = UNSET;
        point.safe_traversal_id = UNSET;
        point.cone_id = 0;
        point.is_cone_point = false;

        row.push_back(point);
        if (row.size() % width == 0)
        {
            result.push_back(row);
            row.clear();
        }
    }

    return result;
}

void remove_edges(std::vector<std::vector<PcPoint_t>> &cloud, int thresh)
{
    int maxY = cloud.size() - thresh;
    int maxX = cloud[0].size() - thresh;
    for (size_t y = 0; y < cloud.size(); y++)
    {
        for (size_t x = 0; x < cloud[y].size(); x++)
        {
            if (y <= thresh 
                    || y >= maxY
                    || x <= thresh
                    || x >= maxX)
            {
                remove_point(cloud[y][x]);
            }
        }
    }
}

void unrotate_cloud(std::vector<std::vector<PcPoint_t>> &cloud) 
{
    float unrotation_angle_rad = -32.0f * DEG_TO_RAD; // 32 degrees
    
    float c = cos(unrotation_angle_rad);
    float s = sin(unrotation_angle_rad);

    for (size_t y = 0; y < cloud.size(); y++) 
    {
        for (size_t x = 0; x < cloud[y].size(); x++)
        {
            float oldY = cloud[y][x].y;
            float oldZ = cloud[y][x].z;
            float oldnY = cloud[y][x].ny;
            float oldnZ = cloud[y][x].nz;

            cloud[y][x].y = (c * oldY) - (s * oldZ);
            cloud[y][x].z = (s * oldY) + (c * oldZ);
            cloud[y][x].ny = (c * oldnY) - (s * oldnZ);
            cloud[y][x].nz = (s * oldnY) + (c * oldnZ);
        }
    }
}

void threshold_normals(std::vector<std::vector<PcPoint_t>> &cloud)
{
    float goodThresh = cos(10 * DEG_TO_RAD); 
    float badThresh = cos(30 * DEG_TO_RAD);

    for (size_t y = 0; y < cloud.size(); y++) 
    {
        for (size_t x = 0; x < cloud[y].size(); x++) 
        {
            if (cloud[y][x].ny >= goodThresh)
            {
                cloud[y][x].normal_threshold_id = SAFE;
            }
            else if (cloud[y][x].ny <= badThresh)
            {
                cloud[y][x].normal_threshold_id = UNSAFE;
            }
            else
            {
                cloud[y][x].normal_threshold_id = BORDERLINE;
            }
        }
    }
}

void floodfill_safe_traversal(std::vector<std::vector<PcPoint_t>> &cloud)
{
    for (size_t y = 0; y < cloud.size(); y++) 
    {
        for (size_t x = 0; x < cloud[y].size(); x++)
        {
            if (cloud[y][x].normal_threshold_id == UNSAFE)
            {
                cloud[y][x].safe_traversal_id = cloud[y][x].normal_threshold_id;
            }
            else
            {
                cloud[y][x].safe_traversal_id = UNSET;
            }
        }
    }

    float minCosDeltaAngle = cos(DEG_TO_RAD * 10);  //Max delta theta between any two points
    float maxDxSq = (4 * IN_TO_M) * (4 * IN_TO_M);  //Max delta in distance between any two points, squared

    std::queue<std::pair<int, int>> queue;

    // Start in the bottom center of the image
    int start_min_y = std::max(0, static_cast<int>(cloud.size() - 200));
    int start_max_y = cloud.size();
    int start_min_x = (cloud[0].size() / 2) - 200;
    int start_max_x = (cloud[0].size() / 2) + 200;
    
    for (int y = start_min_y; y < start_max_y; y++)
    {
        for (int x = start_min_x; x < start_max_x; x++)
        {
            if (cloud[y][x].normal_threshold_id == SAFE)
            {
                cloud[y][x].safe_traversal_id = SAFE;
                queue.emplace(std::make_pair(y, x));
            }
            else if (cloud[y][x].normal_threshold_id == UNSAFE)
            {
                cloud[y][x].safe_traversal_id = UNSAFE;
            }
        }
    }

    // Examine neighbors
    int maxY = cloud.size();
    int maxX = cloud[0].size();
    int dist_reject = 0;
    int angle_reject = 0;

    while (!queue.empty())
    {
        std::pair<int, int> good_point_coords = queue.front();
        queue.pop();

        int y = good_point_coords.first;
        int x = good_point_coords.second;
        int radius = 1;
        PcPoint_t good_point = cloud[y][x];
        
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
                if (ny < 0 || ny >= maxY || nx < 0 || nx >= maxX)
                {
                    continue;
                }

                // Check that we have data for the point and have not already examined it.
                PcPoint_t new_point = cloud[ny][nx];
                if (!is_valid_point(new_point)
                        || new_point.safe_traversal_id != UNSET)
                {
                    continue;
                }

                // Check that the two points are not too far apart.
                // This would indicate a ledge.
                float distance_sq = (new_point.x-good_point.x)*(new_point.x-good_point.x);
                distance_sq += (new_point.y-good_point.y)*(new_point.y-good_point.y);
                distance_sq += (new_point.z-good_point.z)*(new_point.z-good_point.z);
                if (distance_sq > maxDxSq)
                {
                    dist_reject++;
                    continue;
                }

                // Check that the normals aren't too far apart.
                float norm_dot = (new_point.nx*good_point.nx) + (new_point.ny*good_point.ny) + (new_point.nz*good_point.nz);
                if (norm_dot < minCosDeltaAngle)
                {
                    angle_reject++;
                    continue;
                }

                // We have a good point. Set its status as "good" and add it to the queue.
                cloud[ny][nx].safe_traversal_id = SAFE;
                queue.emplace(std::make_pair(ny, nx));
            }
        }
    }

    // Any points at this point are unreachable from a good sector.
    // Mark them as unsafe.
    for (size_t y = 0; y < cloud.size(); y++)
    {
        for (size_t x = 0; x < cloud[0].size(); x++)
        {
            if (cloud[y][x].safe_traversal_id == UNSET)
            {
                cloud[y][x].safe_traversal_id = UNSAFE;
            }
        }
    }

    // TODO: close holes
}

void identify_cone_points(std::vector<std::vector<PcPoint_t>> &cloud)
{
    int point_count = 0;
    for (size_t y = 0; y < cloud.size(); y++)
    {
        for (size_t x = 0; x < cloud[y].size(); x++)
        {
            if (is_valid_point(cloud[y][x])
                    && cloud[y][x].safe_traversal_id == UNSAFE)
            {
                cloud[y][x].is_cone_point = is_cone_color(cloud[y][x].rgba);
                if (cloud[y][x].is_cone_point)
                {
                    point_count++;
                }
            }
        }
    }
}

void cluster_cones(std::vector<std::vector<PcPoint_t>> &cloud)
{
    constexpr float max_grouping_distance_sq = (5 * IN_TO_M) * (5 * IN_TO_M);
    constexpr float max_aspect_ratio = 0.75;
    constexpr float min_point_count = 500;

    uint32_t cone_id_counter = 1;
    for (size_t y = 0; y < cloud.size(); y++)
    {
        for (size_t x = 0; x < cloud[0].size(); x++)
        {
            if (cloud[y][x].is_cone_point && cloud[y][x].cone_id == 0)
            {
                std::unordered_set<int> cone_points;
                int minX = x;
                int maxX = x;
                int minY = y; 
                int maxY = y;
                int width = cloud[0].size();

                std::queue<int> points_to_visit;
                points_to_visit.emplace((y*width) + x);
                cone_points.insert((y*width) + x);

                while (!points_to_visit.empty())
                {
                    int packed = points_to_visit.front();
                    points_to_visit.pop();

                    int y = packed / width;
                    int x = packed % width;
                    PcPoint_t this_point = cloud[y][x];

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
                            int new_packed = (ny*width) + nx;
                            PcPoint_t new_point = cloud[ny][nx];

                            float distance_sq = (new_point.x - this_point.x) * (new_point.x - this_point.x);
                            distance_sq += (new_point.y - this_point.y) * (new_point.y - this_point.y);
                            distance_sq += (new_point.z - this_point.z) * (new_point.z - this_point.z);

                            bool visited = (cone_points.count(new_packed) != 0);

                            if (new_point.is_cone_point 
                                    && new_point.cone_id == 0
                                    && !visited
                                    && distance_sq < max_grouping_distance_sq)
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

                float is_valid_cone_blob = (aspect_ratio <= max_aspect_ratio) && (cone_points.size() >= min_point_count);

                for (int packed : cone_points)
                {
                    int y = packed / width;
                    int x = packed % width;

                    if (is_valid_cone_blob)
                    {
                        cloud[y][x].cone_id = cone_id_counter;
                        cloud[y][x].is_cone_point = true; //??
                    }
                    else
                    {
                        cloud[y][x].cone_id = 0;
                        cloud[y][x].is_cone_point = false;
                    }
                }
                
                if (is_valid_cone_blob)
                {
                    cone_id_counter++;
                }
            }
        }
    }
}

std::vector<std::vector<int>> make_occupancy_matrix(std::vector<std::vector<PcPoint_t>> &cloud)
{
    constexpr float grid_square_size_m = (3.0f * IN_TO_M);
    constexpr float min_num_points = 10;

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();

    for (int y = 0; y < cloud.size(); y++)
    {
        for (int x = 0; x < cloud[0].size(); x++)
        {
            if (is_valid_point(cloud[y][x]))
            {
                min_x = std::min(min_x, cloud[y][x].x);
                max_x = std::max(max_x, cloud[y][x].x);
                min_z = std::min(min_z, cloud[y][x].z);
                max_z = std::max(max_z, cloud[y][x].z);
            }
        }
    }

    int num_squares_wide = static_cast<int>(ceil((max_x - min_x) / grid_square_size_m));
    int num_squares_tall = static_cast<int>(ceil((max_z - min_z) / grid_square_size_m));

    std::vector<std::vector<std::map<int, int>>> point_counters;
    std::vector<std::vector<int>> output;
    point_counters.reserve(num_squares_tall);
    output.reserve(num_squares_tall);
    for (int i = 0; i < num_squares_tall; i++)
    {
        std::vector<std::map<int, int>> tmp;
        std::vector<int> tmp2;
        tmp.reserve(num_squares_wide);
        tmp2.reserve(num_squares_wide);
        for (int j = 0; j < num_squares_wide; j++)
        {
            std::map<int, int> val;
            tmp.push_back(val);
            tmp2.push_back(0);
        }
        point_counters.push_back(tmp);
        output.push_back(tmp2);
    }

    for (int y = 0; y < cloud.size(); y++)
    {
        for (int x = 0; x < cloud[0].size(); x++)
        {
            PcPoint_t p = cloud[y][x];

            if (!is_valid_point(p))
            {
                continue; 
            }

            int occ_x = floor((p.x - min_x) / grid_square_size_m);
            int occ_z = floor((p.z - min_z) / grid_square_size_m);

            if (p.is_cone_point)
            {
                point_counters[occ_z][occ_x][p.cone_id]++;
            }
            else if (p.safe_traversal_id == SAFE)
            {
                point_counters[occ_z][occ_x][0]++;
            }
            else if (p.safe_traversal_id == UNSAFE)
            {
                point_counters[occ_z][occ_x][-1]++;
            }
            else if (p.safe_traversal_id == UNSET)
            {
                point_counters[occ_z][occ_x][-2]++;
            }
        }
    }

    for (int z = 0; z < num_squares_tall; z++)
    {
        for (int x = 0; x < num_squares_wide; x++)
        {
            std::map<int, int> collected_points = point_counters[z][x];

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

            if (max_value < min_num_points)
            {
                output[z][x] = -3;
            }
            else
            {
                output[z][x] = max_key;
            }
        }
    }

    return output;
}

cv::Mat visualize_occupancy_matrix(std::vector<std::vector<int>> &occupancy_matrix)
{
    int max_image_dim_px = 1000;

    int width_block_size = (max_image_dim_px - (max_image_dim_px % occupancy_matrix[0].size())) / occupancy_matrix[0].size();
    int height_block_size = (max_image_dim_px - (max_image_dim_px % occupancy_matrix.size())) / occupancy_matrix.size();

    int block_size = std::min(width_block_size, height_block_size);

    cv::Mat output(occupancy_matrix.size() * block_size,
                    occupancy_matrix[0].size() * block_size,
                    CV_8UC3);

    for (int z = 0; z < occupancy_matrix.size(); z++)
    {
        for (int x = 0; x < occupancy_matrix[0].size(); x++)
        {
            cv::Point upper_left((x*block_size), (z*block_size));
            cv::Point lower_right(((x+1)*block_size), ((z+1)*block_size));

            constexpr int wall_width = 2;
            cv::Scalar color;
            switch (occupancy_matrix[z][x])
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
                    throw std::runtime_error("Unrecognized occupancy matrix value: " + std::to_string(occupancy_matrix[z][x]));
            }

            cv::rectangle(output, upper_left, lower_right, color, -1); //filled
            cv::rectangle(output, upper_left, lower_right, cv::Scalar(0, 0, 0), wall_width); 
        }
    }

    return output;
}

int main(int argc, char** argv) 
{
    std::vector<std::vector<PcPoint_t>> cloud;
    if (argc == 1)
    {
        std::cout << "Reading point cloud from camera..." << std::endl;
        cloud = read_point_cloud_from_camera();
        //cloud = opencv_point_cloud_from_camera();
    }
    else if (argc == 2)
    {
        std::cout << "Reading point cloud from " << argv[1] << "..." << std::endl;
        cloud = read_point_cloud_from_file(std::string(argv[1]));
    }

    std::cout << "Writing to 'raw.pcd'..." << std::endl;
    write_to_pcd(cloud, "raw.pcd");

    std::cout << "Unrotating..." << std::endl;
    unrotate_cloud(cloud);

    std::cout << "Writing to 'unrotated.pcd'..." << std::endl;
    write_to_pcd(cloud, "unrotated.pcd");

    std::cout << "Removing low-confidence points..." << std::endl;
    remove_points_below_confidence(cloud, 50);

    std::cout << "Writing to 'hiconf.pcd'..." << std::endl;
    write_to_pcd(cloud, "hiconf.pcd");

    std::cout << "Applying initial normal thresholding..." << std::endl;
    threshold_normals(cloud);

    std::cout << "Writing to 'thresholded.pcd'..." << std::endl;
    write_to_pcd(cloud, "thresholded.pcd", NORM_THRESH);

    std::cout << "Determining safe traversal regions..." << std::endl;
    floodfill_safe_traversal(cloud);

    std::cout << "Writing to 'safe.pcd'..." << std::endl;
    write_to_pcd(cloud, "safe.pcd", SAFE_TRAVERSAL);

    std::cout << "Identifying cone points..." << std::endl;
    identify_cone_points(cloud);

    std::cout << "Writing to 'conepoints.pcd'..." << std::endl;
    write_to_pcd(cloud, "conepoints.pcd", IS_CONE);

    std::cout << "Grouping cones..." << std::endl;
    cluster_cones(cloud);

    std::cout << "Writing to 'clustered.pcd'..." << std::endl;
    write_to_pcd(cloud, "clustered.pcd", SAFE_TRAVERSAL_WITH_CONE);

    std::cout << "Computing occupancy matrix..." << std::endl;
    std::vector<std::vector<int>> occupancy_matrix = make_occupancy_matrix(cloud);

    std::cout << "Creating occupancy matrix visualization..." << std::endl;
    cv::Mat occupancy_matrix_visualization = visualize_occupancy_matrix(occupancy_matrix);

    std::cout << "Writing visualization to file..." << std::endl;
    cv::imwrite("matrix.bmp", occupancy_matrix_visualization);

    std::cout << "Graceful termination." << std::endl;
    return 0;
}
