#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <stdexcept>
#include <ctype.h>

int main(int argc, char** argv)
{

  	ros::init(argc, argv, "webcam_sensor_reader");
 	ros::NodeHandle nh;
  	image_transport::ImageTransport it(nh);
  	image_transport::Publisher pub = it.advertise("output_topic", 1);

	int channel = 0;
	double frames_per_second = 30;

	int c;
	while((c = getopt(argc, argv, "i:f:")) != -1)
	{
		switch (c)
		{
			case 'i':
				try
				{
					int channel = std::stoi(optarg, NULL);
					break;
				}
				catch (std::exception ex)
				{
					ROS_FATAL(
						"Could not parse channel from provided string %s. The following error was encountered: %s", 
						optarg, 
						ex.what());
					return 1;
				}
			case 'f':
				try
				{
					frames_per_second = std::stod(optarg, NULL);
				}
				catch (std::exception ex)
				{
					ROS_FATAL(
						"Could not parse channel from provided string %s. THe following error was encountered: %s",
						optarg, 
						ex.what());
					return 1;
				}
			case '?':
				ROS_FATAL("Unknow option character %c.", optopt);
				return 1;
			default:
				ROS_FATAL("Something terrible happened while calling getopt.");
				abort();
		}
	}

  	cv::VideoCapture cap(channel);
  	if(!cap.isOpened())
  	{
    	ROS_ERROR("Could not open webcam at channel %d", channel);
    	return 1;
  	}

  	cv::Mat frame;
  	sensor_msgs::ImagePtr msg;

  	ros::Rate loop_rate(frames_per_second); 
  	while (ros::ok())
  	{
    	cap >> frame;

    	if(!frame.empty()) 
    	{
      		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      		pub.publish(msg);
    	}

    	ros::spinOnce();
    	loop_rate.sleep();
  	}
}

