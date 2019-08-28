#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <magellan_messages/MsgVisualOdometryDebug.h>
#include <magellan_messages/MsgVisualOdometry.h>
#include <unistd.h>
#include <stdexcept>
#include <ctype.h>
#include <vector>

ros::Publisher debug_publisher;
ros::Publisher position_publisher;
ros::Subscriber input_subscriber;

cv::Mat last_frame;
std::vector<cv::Point2f> last_frame_corners;
bool last_frame_valid = false;

/* goodFeaturesToTrack parameters */
int max_corners = -1;              // -c
double quality_level = -1;         // -q
double min_distance = -1;          // -d
int block_size = -1;               // -b
int use_harris_detector = -1;      // -h
double harris_k = -1;              // -k

/* calcOpticalFlowPyrLk parameters */
int window_size = -1;              // -w
int max_level = -1;                // -l
double term_epsilon = -1;          // -e
int term_count = -1;               // -n
double min_eig_threshold = -1;     // -t


void message_received_callback(const sensor_msgs::ImageConstPtr &input_data)
{
    cv_bridge::CvImagePtr img_ptr;

    try
    {
        img_ptr = cv_bridge::toCvCopy(input_data, sensor_msgs::image_encodings::BGR8);
    }
    catch (...)
    {
        ROS_ERROR("Exception in conversion.");
        return;
    }

    if (!last_frame_valid)
    {
        cv::cvtColor(img_ptr->image, last_frame, CV_BGR2GRAY);
        cv::goodFeaturesToTrack(
                last_frame, 
                last_frame_corners, 
                max_corners, 
                quality_level,
                min_distance,
                cv::noArray(), //mask - we want to use all points in the image
                block_size,
                use_harris_detector > 0,
                harris_k);
        return;
    }

    cv::Mat current_frame;
    std::vector<cv::Point2f> current_frame_corners;

    cv::cvtColor(img_ptr->image, current_frame, CV_BGR2GRAY);

    cv::TermCriteria tc = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), term_count, term_epsilon);
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
            last_frame, 
            current_frame, 
            last_frame_corners, 
            current_frame_corners, 
            status, 
            error, 
            cv::Size(window_size, window_size), 
            max_level, 
            tc, 
            0, 
            min_eig_threshold);

    // TODO: This is roughly using the code from Avi Sing's blog
    // Verify that it does something useful.


    // TODO: how to pass in calibration parameters.
    cv::Mat essential_mat = cv::findEssentialMat(last_frame_corners, current_frame_corners);

    cv::Mat R;
    cv::Mat t;
    cv::recoverPose(essential_mat, last_frame_corners, current_frame_corners, R, t);

    // TODO: Do something with the recovered pose.
    
    last_frame = current_frame.clone();
    last_frame_corners.swap(current_frame_corners);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_odometry");

    int c;
    while((c = getopt(argc, argv, "c:q:d:b:h:k:w:l:e:n:t:")) != -1)
    {
        switch (c)
        {
            case 'c':
                try
                {
                    max_corners = std::stoi(optarg, NULL);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse max_corners from string %s. Expected integer.",
                            optarg);
                    return 1;
                }
                break;
            case 'q':
                try
                {
                    quality_level = std::stod(optarg);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse quality_level from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case 'd':
                try
                {
                    min_distance = std::stod(optarg);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse min_distance from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case 'b':
                try
                {
                    block_size = std::stoi(optarg, NULL);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse block_size from string %s. Expected int.",
                            optarg);
                    return 1;
                }
                break;
            case 'h':
                try
                {
                    use_harris_detector = std::stoi(optarg, NULL);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse use_harris_detector from string %s. Expected int.",
                            optarg);
                    return 1;
                }
                break;
            case 'k':
                try
                {
                    harris_k = std::stod(optarg);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse harris_k from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case 'w':
                try
                {
                    window_size = std::stoi(optarg, NULL);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse window_size from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case 'l':
                try
                {
                    max_level = std::stoi(optarg, NULL);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse max_level from string %s. Expected int.",
                            optarg);
                    return 1;
                }
                break;
            case 'e':
                try
                {
                    term_epsilon = std::stod(optarg);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse term_epsilon from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case 'n':
                try
                {
                    term_count = std::stoi(optarg, NULL);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse term_count from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case 't':
                try
                {
                    min_eig_threshold = std::stod(optarg);
                }
                catch (...)
                {
                    ROS_FATAL("Could not parse min_eig_threshold from string %s. Expected double.",
                            optarg);
                    return 1;
                }
                break;
            case '?':
            default:
                ROS_FATAL("Unrecognized argument: %c",
                        c);
                return 1;
        }
    }

    if (max_corners < 0)
    {
        ROS_FATAL("max_corners parameter invalid or not supplied. Use -c parameter to specify.");
        return 1;
    }

    if (quality_level < 0)
    {
        ROS_FATAL("quality_level parameter invalid or not specified. Use -q parameter to specify.");
        return 1;
    }

    if (min_distance < 0)
    {
        ROS_FATAL("min_distance parameter invalid or not specified. Use -d parameter to specify.");
        return 1;
    }

    if (block_size < 0)
    {
        ROS_FATAL("block_size parameter invalid or not specified. Use -b parameter to specify.");
        return 1;
    }

    if (use_harris_detector < 0 || use_harris_detector > 1)
    {
        ROS_FATAL("use_harris_detector parameter invalid or not specified. Use -h parameter to specify.");
        return 1;
    }

    if (use_harris_detector && harris_k < 0)
    {
        ROS_FATAL("harris_k parameter invalid or not specified. Use -k parameter to specify.");
        return 1;
    }

    if (window_size < 0)
    {
        ROS_FATAL("window_size parameter invalid or not specified. Use -w parameter to specify.");
        return 1;
    }

    if (max_level < 0)
    {
        ROS_FATAL("max_level parameter invalid or not specified. Use -l parameter to specify.");
        return 1;
    }

    if (term_epsilon < 0)
    {
        ROS_FATAL("term_epsilon parameter invalid or not specified. Use -e parameter to specify.");
        return 1;
    }

    if (term_count < 0)
    {
        ROS_FATAL("term_count parameter invalid or not specified. Use -n parameter to specify.");
        return 1;
    }

    if (min_eig_threshold < 0)
    {
        ROS_FATAL("min_eig_threshold parameter invalid or not specified. Use -t parameter to specify.");
        return 1;
    }

    ros::NodeHandle nh;

    debug_publisher = nh.advertise<magellan_messages::MsgVisualOdometryDebug>("output_topic_debug", 1000);
    position_publisher = nh.advertise<magellan_messages::MsgVisualOdometry>("output_topic", 1000);
    input_subscriber = nh.subscribe("input_topic", 1000, message_received_callback);

    ros::spin();
}
