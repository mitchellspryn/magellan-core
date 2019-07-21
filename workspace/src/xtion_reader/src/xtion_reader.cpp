#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <stdexcept>
#include <ctype.h>

#include <OpenNI.h>

int openStream(openni::Device& device, const char* name, openni::SensorType sensorType, openni::VideoStream& stream, const openni::SensorInfo** ppSensorInfo, bool* pbIsStreamOn)
{
    *ppSensorInfo = device.getSensorInfo(sensorType);
	*pbIsStreamOn = false;

	if (*ppSensorInfo == NULL)
	{
        ROS_FATAL("No %s sensor available\n", name);
        return -1;
	}

	openni::Status nRetVal = stream.create(device, sensorType);
	if (nRetVal != openni::STATUS_OK)
	{
        ROS_FATAL("Failed to create %s stream:\n%s\n", openni::OpenNI::getExtendedError(), name);
        return -2;
	}

	nRetVal = stream.start();
	if (nRetVal != openni::STATUS_OK)
	{
		stream.destroy();

        ROS_FATAL("Failed to start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
        return -3;
	}

	*pbIsStreamOn = true;

	return 0;
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "xtion_reader");

	char* deviceId = NULL;
	double frames_per_second = 30;
    int queue_size = 1;

	int c;
	while((c = getopt(argc, argv, "i:f:q:")) != -1)
	{
		switch (c)
		{
			case 'i':
				try
				{
                    int len = strlen(optarg);
                    deviceId = (char*)calloc(len, sizeof(char));
                    strcpy(deviceId, optarg);
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
            case 'q':
                try
                {
                    queue_size =  std::stoi(optarg, NULL);
                    break; 
                }
                catch (std::exception ex)
                {
                    ROS_FATAL(
						"Could not parse queue_size from provided string %s. The following error was encountered: %s", 
						optarg, 
						ex.what());
                }
			case '?':
				ROS_FATAL("Unknow option character %c.", optopt);
				return 1;
			default:
				ROS_FATAL("Something terrible happened while calling getopt.");
				abort();
		}
	}

    if (deviceId == NULL)
    {
        ROS_FATAL("Could not identify the deviceId. Please pass -i parameter.");
        abort();
    }

    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        ROS_FATAL("Could not initialize OpenNI.");
        abort();
    }

    openni::Device device;
    openni::VideoStream colorStream;
    openni::VideoStream depthStream;
    const openni::SensorInfo* colorSensorInfo = NULL;
    const openni::SensorInfo* depthSensorInfo = NULL;
    openni::PlaybackControl* playbackControl = NULL;
    bool depthOn = false;
    bool colorOn = false;

    rc = device.open(deviceId);
    if (rc != openni::STATUS_OK)
    {
        ROS_FATAL("Could not open device with URI %s", deviceId);
        abort();
    }

    playbackControl = device.getPlaybackControl();
    int openResult = openStream(device, "depth", openni::SENSOR_DEPTH, depthStream, &depthSensorInfo, &depthOn);
    if (openResult != 0)
    {
        ROS_FATAL("Could not open the depth sensor.");
        abort();
    }

    openResult = openStream(device, "color", openni::SENSOR_COLOR, colorStream, &colorSensorInfo, &depthOn);
    if (openResult != 0)
    {
        ROS_FATAL("Could not open the color sensor.");
        abort();
    }

    rc = device.setDepthColorSyncEnabled(true);
    if (rc != openni::STATUS_OK)
    {
        ROS_FATAL("Could not sync depth and color sensors.");
        abort();
    }

 	ros::NodeHandle nh;
  	image_transport::ImageTransport it(nh);
  	image_transport::Publisher rgb_pub = it.advertise("rgb_output_topic", queue_size);
  	image_transport::Publisher depth_pub = it.advertise("depth_output_topic", queue_size);

    free(deviceId);

    // TODO: open frames	

    openni::VideoFrameRef rgbFrameRef;
    openni::VideoFrameRef depthFrameRef;
  	sensor_msgs::ImagePtr msg;
    bool newRgb = false;
    bool newDepth = false;
    bool opened = false;
    int failCount = 0;

    openni::VideoStream* streams[] {&colorStream, &depthStream};

  	ros::Rate loop_rate(frames_per_second); 
  	while (ros::ok())
  	{
        int changedIndex = -1;
        rc = openni::OpenNI::waitForAnyStream(streams, 2, &changedIndex, 50); //TODO: make parameter

        if (rc == openni::STATUS_TIME_OUT)
        {
            // It takes a few seconds for the camera to come up.
            // Avoid printing errors to stdout until we've connected successfully at least once, or we've tried 100 times.
            failCount++;
            if (opened || failCount > 100)
            {
                ROS_ERROR("Timed out waiting for stream.");
            }
        }
        else if (rc != openni::STATUS_OK)
        {
            ROS_FATAL("Unsuccessful status code returned from WaitForAnyStream: %d", rc);
            abort();
        }
        else
        {
            switch (changedIndex)
            {
                case 0:
                    colorStream.readFrame(&rgbFrameRef);
                    newRgb = true;
                    break;
                case 1:
                    depthStream.readFrame(&depthFrameRef);
                    newDepth = true;
                    break;
                default:
                    ROS_FATAL("Unrecognized index returned from WaitForAnyStream: %d", changedIndex);
                    abort();
            }

            if (newRgb && newDepth)
            {
                cv::Mat cvDepthFrame(depthFrameRef.getHeight(), depthFrameRef.getWidth(), CV_16UC1, (void*)depthFrameRef.getData());
                cv::Mat cvVideoFrame(rgbFrameRef.getHeight(), rgbFrameRef.getWidth(), CV_8UC3, (void*)rgbFrameRef.getData());

                sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", cvDepthFrame).toImageMsg();
                sensor_msgs::ImagePtr videoMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvVideoFrame).toImageMsg();

                opened = true;

                videoMsg->header = depthMsg->header;

                depth_pub.publish(depthMsg);
                rgb_pub.publish(videoMsg);

                newRgb = false;
                newDepth = false;

    	        loop_rate.sleep();
            }
        }

    	ros::spinOnce();
  	}
}

