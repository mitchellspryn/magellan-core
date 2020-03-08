#!/usr/bin/env python3
import flask
import os
import rospy
import signal
import warning
import cv2
import threading
import base64
import datetime
import numpy as np
import pandas as pd
import math
import json
import requests

from sensor_msgs.msg import Image, LaserScan, NavSatFix
from magellan_messages.msg import MsgMagellanImu, MsgMagellanDrive, MsgMagellanImu
from cv_bridge import CvBridge, CvBridgeError

app = flask.Flask(__name__)

publisher_xtion_depth = None
publisher_xtion_rgb = None
publisher_lidar = None
publisher_bottom_camera = None
publisher_imu = None
publisher_gps = None
publisher_run_complete_callback = None

subscriber_control_signals = None

cv_bridge = CvBridge()

@app.route('/ping')
def ping():
    return 'pong', 200

@app.route('/xtion')
def receive_xtion():
    global publisher_xtion_depth
    global publisher_xtion_rgb
    global cv_bridge

    data = flask.request.json
    rgb_shape = data['rgb_shape']
    depth_shape = data['depth_shape']

    rgb_image = np.fromstring(data['rgb_data'], order='C', dtype=np.uint8).reshape(rgb_shape)
    depth_image = np.fromstring(data['depth_data'], order='C', dtype=np.float32).reshape(depth_shape)

    now = rospy.Time.now()
    rgb_image_msg = cv_bridge.cv2_to_imgmsg(rgb_image, 'bgr8')[::-1]
    rgb_image_msg.stamp.header = now

    depth_image_msg = cv_bridge.cv2_to_imgmsg(depth_image, 'mono16')
    depth_image_msg.stamp.header = now

    publisher_xtion_rgb.publish(rgb_image_msg)
    publisher_xtion_depth.publisher(depth_image_msg)

    return '', 200

@app.route('/lidar')
def receive_lidar():
    global publisher_lidar
    
    data = flask.request.json
    data_shape = data['shape']
    data_ranges = np.fromstring(data['ranges'], dtype=np.float32).reshape(data_shape)
    intensities = np.zeros(shape=data_shape, dtype=np.float32)

    lidar_scan = LidarScan()
    lidar_scan.range_min = 0
    lidar_scan.range_max = 6.28 # 2*PI
    lidar_scan.angle_increment = 6.28 / data_shape[0] 
    lidar_scan.intensities = intensities
    lidar_scan.ranges = data_ranges
    lidar_scan.range_max = 6000
    lidar_scan.range_min = 150

    lidar_scan.header.stamp = rospy.Time.now()

    publisher_lidar.publish(lidar_scan)

    return '', 200

@app.route('/ground_camera')
def receive_ground_camera():
    global publisher_ground_camera
    global cv_bridge

    data = flask.request.json
    shape = data['shape']
    rgb = np.fromstring(data['data'], order='C', dtype=np.uint8)

    now = rospy.Time.now()
    img = cv_bridge.cv2_to_imgmsg(rgb, 'bgr8')[::-1]
    img.stamp.header = now

    publisher_ground_camera.publish(img)

    return '', 200

@app.route('/imu_gps')
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
    gps_msg.stamp.header = now

    gps_msg.latitude = data['latitude']
    gps_msg.longitude = data['longitude']
    gps_msg.altitude = data['altitude']

    publisher_imu.publish(imu_msg)
    publisher_gps.publish(gps_msg)

@app.route('/run_complete')
def run_complete_callback():
    # TODO: do something with this
    return '', 200

def process_control_signals(drive_msg):
    data = {}
    data['left_throttle'] = drive_msg.left_throttle
    data['right_throttle'] = drive_msg.right_throttle

    requests.post('http://localhost:12345/drive', json=data)

def sig_handler(sig, frame):
    raise KeyboardInterrupt('ctrl+c pressed')

def main():
   global publisher_xtion_depth
   global publisher_xtion_rgb
   global publisher_lidar
   global publisher_bottom_camera
   global publisher_imu
   global publisher_gps
   global publisher_run_complete_callback
   global subscriber_control_signals

   rospy.init_node('interface_callbacks')

   publisher_xtion_depth = rospy.Publisher('output_xtion_depth', Image, queue_size=100)
   publisher_xtion_rgb = rospy.Publisher('output_xtion_rgb', Image, queue_size=100)
   publisher_lider = rospy.Publisher('output_lidar', LaserScan, queue_size=100)
   publisher_bottom_camera = rospy.Publisher('output_bottom_camera', Image, queue_size=100)
   publisher_imu = rospy.Publisher('output_imu', MsgMagellanImu, queue_size=100)
   publisher_gps = rospy.Publisher('output_gps', NavSatFix, queue_size=100)

   subscriber_control_signals = rospy.Subscriber('input_control_signals', MsgMagellanDrive, queue_size=100)

   signal.signal(signal.SIGINT, sig_handler)

   app.run(host='0.0.0.0', port=55555)

if __name__ == '__main__':
    main()
