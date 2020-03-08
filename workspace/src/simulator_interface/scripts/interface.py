#!/usr/bin/env python3
import flask
import os
import rospy
import signal
import cv2
import threading
import base64
import datetime
import numpy as np
import pandas as pd
import math
import json
import requests
import sys
import argparse

from sensor_msgs.msg import Image, LaserScan, NavSatFix
from magellan_messages.msg import MsgMagellanImu, MsgMagellanDrive, MsgMagellanImu
from cv_bridge import CvBridge, CvBridgeError

app = flask.Flask(__name__)

publisher_xtion_depth = None
publisher_xtion_rgb = None
publisher_lidar = None
publisher_ground_camera = None
publisher_imu = None
publisher_gps = None
publisher_run_complete_callback = None
simulator_uri = None

subscriber_control_signals = None

cv_bridge = CvBridge()

@app.route('/ping', methods=['GET'])
def ping():
    return 'pong', 200

@app.route('/xtion', methods=['POST'])
def receive_xtion():
    global publisher_xtion_depth
    global publisher_xtion_rgb
    global cv_bridge

    data = flask.request.get_data(parse_form_data=False)
    height = int.from_bytes(data[0:4], 'big')
    width = int.from_bytes(data[4:8], 'big')

    num_rgb_bytes = height * width * 4
    num_depth_bytes = height * width * 2

    rgb_start = 8
    rgb_end = 8+num_rgb_bytes

    rgb_image = np.frombuffer(data[rgb_start:rgb_end], dtype=np.uint8).reshape((height, width, 4))[:, :, 0:3] # remove alpha channel
    depth_image = np.frombuffer(data[rgb_end:], dtype=np.uint16).reshape((height, width, 1)) 

    # TODO: for some reason, the RGB image comes in upside-down
    # This doesn't appear to affect the depth image.
    now = rospy.Time.now()
    rgb_image_msg = cv_bridge.cv2_to_imgmsg(np.flipud(rgb_image[::-1]), 'bgr8')
    rgb_image_msg.header.stamp = now

    depth_image_msg = cv_bridge.cv2_to_imgmsg(depth_image, 'mono16')
    depth_image_msg.header.stamp = now

    publisher_xtion_rgb.publish(rgb_image_msg)
    publisher_xtion_depth.publish(depth_image_msg)

    return '', 200

@app.route('/lidar', methods=['POST'])
def receive_lidar():
    global publisher_lidar
    
    data = flask.request.get_data(parse_form_data=False)
    data_ranges = np.frombuffer(data, dtype=np.float32).reshape(-1)
    intensities = np.zeros(shape=data_ranges.shape[0], dtype=np.float32)

    lidar_scan = LaserScan()
    lidar_scan.range_min = 0
    lidar_scan.range_max = 6.28 # 2*PI
    lidar_scan.angle_increment = 6.28 / data_ranges.shape[0]
    lidar_scan.intensities = intensities
    lidar_scan.ranges = data_ranges
    lidar_scan.range_max = 6000
    lidar_scan.range_min = 150

    lidar_scan.header.stamp = rospy.Time.now()

    publisher_lidar.publish(lidar_scan)

    return '', 200

@app.route('/ground_camera', methods=['POST'])
def receive_ground_camera():
    global publisher_ground_camera
    global cv_bridge

    data = flask.request.get_data(parse_form_data=False)
    height = int.from_bytes(data[0:4], 'big')
    width = int.from_bytes(data[4:8], 'big')

    rgb = np.frombuffer(data[8:], dtype=np.uint8).reshape((height, width, 4))[:, :, 0:3]

    now = rospy.Time.now()
    img = cv_bridge.cv2_to_imgmsg(np.flipud(rgb[::-1]), 'bgr8')
    img.header.stamp = now

    publisher_ground_camera.publish(img)

    return '', 200

@app.route('/imu_gps', methods=['POST'])
def receive_imu_gps():
    global publisher_imu
    global publisher_gps

    data = flask.request.json

    now = rospy.Time.now()
    imu_msg = MsgMagellanImu()
    imu_msg.header.stamp = now

    imu_msg.imu.linear_acceleration.x = data['imu_x']
    imu_msg.imu.linear_acceleration.y = data['imu_y']
    imu_msg.imu.linear_acceleration.z = data['imu_z']
    imu_msg.imu.angular_velocity.x = data['imu_roll']
    imu_msg.imu.angular_velocity.y = data['imu_pitch']
    imu_msg.imu.angular_velocity.z = data['imu_yaw']
    imu_msg.magnetometer.x = data['mag_x']
    imu_msg.magnetometer.y = data['mag_y']
    imu_msg.magnetometer.z = data['mag_z']

    gps_msg = NavSatFix()
    gps_msg.header.stamp = now

    gps_msg.latitude = data['latitude']
    gps_msg.longitude = data['longitude']
    gps_msg.altitude = data['altitude']

    publisher_imu.publish(imu_msg)
    publisher_gps.publish(gps_msg)

    return '', 200

@app.route('/run_complete', methods=['POST'])
def run_complete_callback():
    # TODO: do something with this
    rospy.logerr('Run complete.')
    return '', 200

@app.route('/run_start', methods=['POST'])
def run_start_callback():
    #TODO: do something with this.
    # It will have information like locations of cones, etc.
    rospy.logerr('Run started.')
    rospy.logerr('Parameters: {0}'.format(str(flask.request.json)))
    return '', 200

def process_control_signals(drive_msg):
    global simulator_uri

    data = {}
    data['left_throttle'] = drive_msg.left_throttle / 100.0
    data['right_throttle'] = drive_msg.right_throttle / 100.0

    output_uri = '{0}/control'.format(simulator_uri)

    requests.post(output_uri, json=data)

def sig_handler(sig, frame):
    raise KeyboardInterrupt('ctrl+c pressed')

def parse_args():
    filtered_args = rospy.myargv(sys.argv)

    parser = argparse.ArgumentParser(description='Interface to mock hardware components to simulator.')

    parser.add_argument('--simulator-uri', dest='simulator_uri', type=str, help='The URI of the simulator (e.g. http://192.168.0.30:5000)')

    return parser.parse_args(filtered_args[1:])

def main():
    global publisher_xtion_depth
    global publisher_xtion_rgb
    global publisher_lidar
    global publisher_ground_camera
    global publisher_imu
    global publisher_gps
    global publisher_run_complete_callback
    global subscriber_control_signals
    global simulator_uri

    rospy.init_node('simulator_interface')

    args = parse_args()
    simulator_uri = args.simulator_uri

    if (simulator_uri.endswith('/')):
       simulator_uri = simulator_uri[:-1]

    publisher_xtion_depth = rospy.Publisher('output_xtion_depth', Image, queue_size=100)
    publisher_xtion_rgb = rospy.Publisher('output_xtion_rgb', Image, queue_size=100)
    publisher_lidar = rospy.Publisher('output_lidar', LaserScan, queue_size=100)
    publisher_ground_camera = rospy.Publisher('output_bottom_camera', Image, queue_size=100)
    publisher_imu = rospy.Publisher('output_imu', MsgMagellanImu, queue_size=100)
    publisher_gps = rospy.Publisher('output_gps', NavSatFix, queue_size=100)

    subscriber_control_signals = rospy.Subscriber('input_control_signals', MsgMagellanDrive, process_control_signals, queue_size=100)

    signal.signal(signal.SIGINT, sig_handler)

    app.run(host='0.0.0.0', port=55555)

if __name__ == '__main__':
    main()
