#!/bin/bash
rsync -avP ~/.ros/*.bag mitchell@192.168.8.174:jetson_rosbags/
