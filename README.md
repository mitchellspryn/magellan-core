# Magellan-Core

## Overview
The Magellan Project is a repo hosting design and software files for an open-source autonomous robotics platform. For more information about the purpose of this project, read the blog posts here:

* **[Introduction to the Platform](http://www.mitchellspryn.com/2019/08/05/Robust-Offroad-Rover-For-Sim2Real-Applications.html)**
* **[Introduction to the included Algorithms](http://www.mitchellspryn.com/2020/09/25/Autonomous-Navigation-In-Outdoor-Environments.html)**

## What's included
The content is divided into a few subdirectories:
* **Analysis**: A few jupyter notebooks used for analyzing some data during the design. In particular, these were useful in debugging the causes of failure of the IMU and GPS to perform due to motor interference.
* **AVR**: This is the code that runs on the arduino. It creates a serial client that continuously reads the IMU and the GPS, which is then read the by the Arduino Serial Reader node to inject the data into ROS. For details about compiling and running this project, see the README.md in that folder.
* **Design**: This directory contains design files for the chassis. In addition, it has some CAD files that were used when laying out the bot.
* **Experimental**: Some experimental programs used while debugging and developing algorithms. These programs are generally one-off programs, which are used to quickly prototype an algorithm. If successful, it is implemented more robustly in the production workspace.
* **Simulation**: Contains some experimental bindings to [UrdfSim](https://github.com/mitchellspryn/UrdfSim) and the [RoboMagellanOrchestrator](https://github.com/mitchellspryn/RoboMagellanOrchestrator). They are functional but do not support all of the sensors currently on the bot.
* **Workspace**: Contains the control system for the robot. For more information about the modules included, see the README.md in this folder.
