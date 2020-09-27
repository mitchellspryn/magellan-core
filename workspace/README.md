# Included Modules
The following modules are included, and are located in the /src folder:

* **arduino_sensor_reader**: This module reads the GPS and IMU data from the arduino, and publishes them to topics. 
* **cone_detector**: An experimental deep learning-based cone detector based off of RGB images. Has been deprecated due to computational cost, low accuracy, and the fact that the primary sensor delivers a point cloud.
* **control_webpage**: Contains the main UI for debugging the autonomous control system. Exposes topics that accept sensor data, and a topic for controlling the motors.
* **drive_from_file**: A node that will drive the motor from a "drive file," which is a list of (left_motor_velocity, right_motor_velocity, time_in_seconds) tuples. This node reads in a text file with these tuples and publishes them to a topic, which can be consumed by the motor control nodes. For examples of these files, look at [/src/drive_from_file/scripts/drive_files](https://github.com/mitchellspryn/magellan-core/tree/master/workspace/src/drive_from_file/scripts/drive_files)
* **localization**: Contains an experimental Kalman filter module. Functional, but useless due to issues with the IMU. See [the blog post](http://www.mitchellspryn.com/2020/09/25/Autonomous-Navigation-In-Outdoor-Environments.html) for more information.
* **magellan_drive_hardware**: Contains the node used for interfacing with the roboclaw motor controllers. Installs a signal handler that will stop the motors if the system is shut down. 
* **magellan_messages**: Contains all of the custom messages used in the system. 
* **magellan_rviz**: Contains an experimental module that took sensor data and displayed it in rviz. This has been deprecated because rviz is quite slow and can't handle point clouds in real time.
* **magellan_serial_port**: A wrapper node that accepts data from a /dev/ttyACM serial port and places it on a topic. This is used by various other nodes that read sensor data via serial ports.
* **motor_ramp**: Generally, it is a bad idea to instantly change the velocity of the motors. This node provides temporal smoothing for any motor control signals. It is intended to be placed between nodes emitting motor control commands (e.g. the planner) and the consumers (e.g. the drive_hardware). 
* **obstacle_detection**: Contains the perception stack. Contains a ros node that accepts point clouds and detects obstacles as described in the [blog post](http://www.mitchellspryn.com/2020/09/25/Autonomous-Navigation-In-Outdoor-Environments.html). In addition, contains a node that can generate debug visualizations from a point cloud stored in an ASCII .pcd format on disk (e.g. one generated from the zed_sensor_reader node). 
* **planner**: Contains the logic responsible for determining the control signals to emit during autonomous operation. It consumes the output of the perception stack and does the world mapping and path planning necessary to determine the proper course of action to take. 
* **rplidar_sensor_reader**: Contains a node that reads the rplidar and publishes the data to a topic.
* **simulator_interface**: Contains an experimental interface to the [UrdfSim](https://github.com/mitchellspryn/UrdfSim) simulator. Currently functional, but missing some sensor models.
* **vision_opencv**: A fork of the [cv_bridge](https://github.com/ros-perception/vision_opencv) project modified to support python3.
* **visual_odometry**: An experimental node that would take in an image from a downward-facing webcam and generate a motion vector. Deprecated as the output of this node is quite noisy due to the large amounts of motion blur present in the webcam image. 
* **webcam_sensor_reader**: Contains a node that can read a USB webcam and publish the image to a topic. 
* **xtion_reader**: Contains a node that can read the depth and RGB images from the [ASUS Xtion PRO](https://www.asus.com/3D-Sensor/Xtion_PRO/) and publish them to ROS topics.
* **zed_sensor_reader**: Contains a node that can read the ZED Sensor reader and publish point clouds, poses, and internal sensor (e.g. IMU) readings to topics. Also contains a node with the capability to generate a point cloud on disk in the ASCII .pcd format.

# Building
To build, first ensure that the following dependencies are installed:
* PCL-1.11 or later
* CUDA 10+ (a GPU is needed to run the ZED2). 

Then, source the rebuild script:
```
. ./rebuild.sh
```

This will build the code, source the ROS environment, and, if run on the jetson, check for the attached hardware and generate the launch files from the launch template files. 

Once built successfully, an incremental rebuild can be run by sourcing the qbuild script:
```
. ./qbuild.sh
```

# Running
To run, it is generally recommended to use the included launch files. Some recipes that are useful:

## Run the full autonomous control system, but don't let the bot move:
```
roslaunch launch/autonomous.launchtemplate
```

This will open up the control webpage at <robot_ip>:12345

## Run the full autonomous control system
```
roslaunch launch/autonomous_drive.launchtemplate
```

This will open up the control webpage at <robot_ip>:12345

## Drive in a predefined path and save collected data to a rosbag
```
roslaunch launch/test_drive_gather_data.launchtemplate drive_file_path:=<drive_file_name>
```

Bag will be saved on the mounted HDD on the jetson.

## Collect a point cloud from the zed
```
rosrun zed_sensor_reader pcd
```

PCD will be saved to "out.pcd"

## Run the perception stack on a pcd file
```
rosrun obstacle_detection debug -i out.pcd
```

This will generate the following visualizations:
* **echo.pcd**: The original file, echoed out to disk.
* **debug.pcd**: The voxelized cloud. Points that are traversable are marked in green, untraversable are red, and cones are in various other colors (purple, yellow, etc). 
* **matrix.bmp**: The generated occupancy matrix. This is what is emitted and consumed by the planner.

The clouds can be viewed with pcl_viewer:
```
pcl_viewer cloud.pcd
```

To view the normals, use the normals flag
```
pcl_viewer cloud.pcd -normals 1 -normals_scale 0.1
```
