#!/usr/bin/env python3
import flask
import os
import rospy
import signal
import warnings
import cv2
import threading
import base64
import datetime
import numpy as np
import pandas as pd
import math
import json

from sensor_msgs.msg import Image, LaserScan, NavSatFix
from magellan_messages.msg import MsgMagellanImu, MsgMagellanDrive, MsgMagellanImu
from cv_bridge import CvBridge, CvBridgeError

status_webpage = None
dummy_image = None
dummy_background_thread = None
app = flask.Flask(__name__)

dummy_image_data = 'data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg=="'
image_xtion_rgb = dummy_image_data
image_xtion_depth = dummy_image_data
image_lidar = dummy_image_data
image_bottom_camera = dummy_image_data
imu_data = {}
gps_data = {}

image_xtion_rgb_lock = threading.Lock()
image_xtion_depth_lock = threading.Lock()
image_lidar_lock = threading.Lock()
image_bottom_camera_lock = threading.Lock()
imu_data_lock = threading.Lock()
gps_data_lock = threading.Lock()

image_xtion_rgb_cvbridge = CvBridge()
image_xtion_depth_cvbridge = CvBridge()
image_bottom_camera_cvbridge = CvBridge()

publisher_motor_controller = None

last_client_request_time = datetime.datetime.utcnow()
pause_request_time = datetime.timedelta(seconds=5)

subscriber_imu = None

def cv_to_byte_stream(frame):
    ret, jpg = cv2.imencode('.jpeg', frame)
    return 'data:image/jpeg;charset=utf-8;base64,' + base64.b64encode(jpg).decode('utf-8')

def process_bottom_image(data):
    global image_bottom_camera
    global image_bottom_camera_lock
    global image_bottom_camera_cvbridge
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return 

    with image_bottom_camera_lock:
        img = image_bottom_camera_cvbridge.imgmsg_to_cv2(data, 'bgr8')[...,::-1]
        image_bottom_camera = cv_to_byte_stream(img)

def process_xtion_rgb_image(data):
    global image_xtion_rgb
    global image_xtion_rgb_lock
    global image_xtion_rgb_cvbridge
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return

    with image_xtion_rgb_lock:
        img = image_xtion_rgb_cvbridge.imgmsg_to_cv2(data, 'bgr8')[...,::-1]
        image_xtion_rgb = cv_to_byte_stream(img)

def process_xtion_depth_image(data):
    global image_xtion_depth
    global image_xtion_depth_lock
    global image_xtion_depth_cvbridge
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return

    with image_xtion_depth_lock:
        img = image_xtion_depth_cvbridge.imgmsg_to_cv2(data, 'mono16')
        img = np.clip(img, 0, 3500) #3.5 m max range at 1mm resolution

        proxy_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        proxy_image[:, :, 1] = (255.0 * (1.0 - (img / 3500.0))).astype(np.uint8)
        proxy_image[:, :, 2] = (255.0 * (1.0 - (img / 3500.0))).astype(np.uint8)
        
        image_xtion_depth = cv_to_byte_stream(proxy_image)

def process_lidar(data):
    global image_lidar
    global image_lidar_lock
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return

    with image_lidar_lock:
        image_size = 480
        output_image = np.zeros((image_size, image_size, 3), dtype=np.uint8)

        for i in range(0, len(data.ranges), 1):
            this_point_angle = float(i) * data.angle_increment
            this_point_range = max(min(data.ranges[i], data.range_max), data.range_min)
            this_point_intensity_red = int(255.0 * (1.0 - (data.intensities[i] / 255)))
            this_point_intensity_blue = int(255.0 * (data.intensities[i] / 255))

            this_point_pixel_distance = (image_size / 2.0) * (this_point_range / data.range_max)
            this_pixel_dx = math.cos(this_point_angle) * this_point_pixel_distance
            this_pixel_dy = math.sin(this_point_angle) * this_point_pixel_distance

            cv2.circle(output_image, (int(this_pixel_dx + (image_size / 2.0)), int(this_pixel_dy + (image_size / 2.0))), 3, (this_point_intensity_red, 0, this_point_intensity_blue), -1)

        cv2.circle(output_image, (int(image_size / 2), int(image_size / 2)), 5, (0, 255, 0), -1)

        image_lidar = cv_to_byte_stream(output_image)

def process_imu(data):
    global imu_data
    global imu_data_lock
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return

    with imu_data_lock:
        imu_data['ax'] = data.imu.linear_acceleration.x
        imu_data['ay'] = data.imu.linear_acceleration.y
        imu_data['az'] = data.imu.linear_acceleration.z

        imu_data['gx'] = data.imu.angular_velocity.x
        imu_data['gy'] = data.imu.angular_velocity.y
        imu_data['gz'] = data.imu.angular_velocity.z

        imu_data['mx'] = data.magnetometer.x
        imu_data['my'] = data.magnetometer.y
        imu_data['mz'] = data.magnetometer.z

def process_gps(data):
    global gps_data
    global gps_data_lock
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return

    with gps_data_lock:
        gps_data['latitude'] = data.latitude
        gps_data['longitude'] = data.longitude
        gps_data['altitude'] = data.altitude

@app.route('/')
def control():
    global status_webpage
    return status_webpage

@app.route('/static/<filename>')
def static_file(filename):
    print(filename)
    return flask.send_from_directory(os.getcwd(), filename)

@app.route('/motor', methods=['POST'])
def set_motor_controls():
    global publisher_motor_controller
    motor_control_data = flask.request.get_json(force=True)

    message = MsgMagellanDrive()
    message.left_throttle = float(motor_control_data['left_throttle'])
    message.right_throttle = float(motor_control_data['right_throttle'])

    publisher_motor_controller.publish(message)

    # Return dummy response to make the framework happy
    return json.dumps({'success':True}), 200, {'ContentType':'application/json'}

@app.route('/data')
def get_cached_topic_data():
    global image_bottom_camera
    global image_xtion_rgb
    global image_xtion_depth
    global image_lidar
    global imu_data
    global gps_data
    global last_client_request_time

    last_client_request_time = datetime.datetime.utcnow()

    response = {}
    response['image_bottom_camera'] = image_bottom_camera
    response['image_xtion_rgb'] = image_xtion_rgb
    response['image_xtion_depth'] = image_xtion_depth
    response['image_lidar'] = image_lidar
    
    response_imu = {}
    for key in ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz']:
        if key in imu_data:
            response_imu[key] = imu_data[key]
        else:
            response_imu[key] = None

    response['imu'] = response_imu

    response_gps = {}
    for key in ['longitude', 'latitude', 'altitude']:
        if key in gps_data:
            response_gps[key] = gps_data[key]
        else:
            response_gps[key] = None

    response['gps'] = response_gps 

    return flask.jsonify(response)

# Use this to cleanly kill the webpage when ctrl+c is pressed
def sig_handler(sig, frame):
    raise KeyboardInterrupt('ctrl+c pressed')

def main():
    global status_webpage
    global publisher_motor_controller
    global subscriber_imu

    rospy.init_node('control_webpage')

    subscriber_image_bottom_camera = rospy.Subscriber('input_image_bottom_camera', Image, process_bottom_image, queue_size=1)
    subscriber_image_xtion_rgb = rospy.Subscriber('input_image_xtion_rgb', Image, process_xtion_rgb_image, queue_size=1)
    subscriber_image_xtion_depth = rospy.Subscriber('input_image_xtion_depth', Image, process_xtion_depth_image, queue_size=1)
    subscriber_lidar = rospy.Subscriber('input_lidar', LaserScan, process_lidar, queue_size=1)
    subscriber_imu = rospy.Subscriber('input_imu', MsgMagellanImu, process_imu, queue_size=1)
    subscriber_gps = rospy.Subscriber('input_gps', NavSatFix, process_gps, queue_size=1)

    publisher_motor_controller = rospy.Publisher('output_motor_control', MsgMagellanDrive, queue_size=1)

    signal.signal(signal.SIGINT, sig_handler)

    with open('control_webpage.html', 'r') as f:
        status_webpage = f.read()

    app.run(host='0.0.0.0', port=12345)

if __name__ == '__main__':
    main()
