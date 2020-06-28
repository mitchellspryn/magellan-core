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
from nav_msgs.msg import Path
from magellan_messages.msg import MsgMagellanDrive, MsgMagellanImu, MsgMagellanPlannerDebug
from cv_bridge import CvBridge, CvBridgeError

status_webpage = None
dummy_image = None
dummy_background_thread = None
app = flask.Flask(__name__)

dummy_image_data = 'data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg=="'
planner_debug_image = dummy_image_data
planner_debug_data = {'debug_image': planner_debug_image}

planner_debug_lock = threading.Lock()

publisher_motor_controller = None

last_client_request_time = datetime.datetime.utcnow()
pause_request_time = datetime.timedelta(seconds=5)

subscriber_planner_debug = None

def cv_to_byte_stream(frame):
    ret, jpg = cv2.imencode('.jpeg', frame)
    return 'data:image/jpeg;charset=utf-8;base64,' + base64.b64encode(jpg).decode('utf-8')

def draw_planner_debug_image(data):
    image_size_px = 800
    image = np.zeros((image_size_px, image_size_px, 3), dtype=np.uint8)

    num_cells_wide = data.global_obstacle_map.map_metadata.width
    num_cells_tall = data.global_obstacle_map.map_metadata.height

    larger_size_cells = max(num_cells_tall, num_cells_wide)
    
    if (larger_size_cells == 0):
        raise ValueError('Received empty grid in draw_planner_debug_image()')

    grid_size_px = int(image_size_px / larger_size_cells)
    max_width = num_cells_wide * grid_size_px
    max_height = num_cells_tall * grid_size_px
    
    origin = data.global_obstacle_map.map_metadata.origin.position
    resolution = data.global_obstacle_map.map_metadata.resolution
    m_to_px = resolution * grid_size_px
    waypoint_size = 10

    # Color each cell according to global grid
    for y in range(0, num_cells_wide, 1):
        for x in range(0, num_cells_tall, 1):
            status = data.global_obstacle_map.matrix[(y * num_cells_tall) + x]

            if (status != 0):
                if (status == -1):
                    color = (0, 0, 0)
                elif (status == 1):
                    color = (255, 0, 255)
                elif (status == 2):
                    color = (0, 255, 255)
                elif (status == 3):
                    color = (255, 255, 0)
                elif (status == 4):
                    color = (255, 128, 255)

                pt1 = (y*grid_size_px, x*grid_size_px)
                pt2 = ((y+1)*grid_size_px, (x+1)*grid_size_px)

                cv2.rectangle(image, pt1, pt2, color, -1)


    # Draw the grid lines
    for y in range(0, num_cells_wide, 1):
        cv2.line(image, (y*grid_size_px, 0), ((y+1)*grid_size_px, max_height), (0, 0, 0), 2)
    for x in range(0, num_cells_tall, 1):
        cv2.line(image, (0, x*grid_size_px), (max_width, (x+1)*grid_size_px), (0, 0, 0), 2)

    # Draw the waypoints
    for waypoint in data.path.poses:
        point_x = (waypoint.pose.position.x - origin.x) * m_to_px
        point_y = (waypoint.pose.position.y - origin.y) * m_to_px

        cv2.circle(image, (point_y, point_x), waypoint_size, (0, 255, 0), -1)

    # Draw the goal point
    goal_x = (data.goal.x - origin.x) * m_to_px
    goal_y = (data.goal.y - origin.y) * m_to_px
    cv2.circle(image, (point_y, point_x), waypoint_size, (255, 0, 0), -1)

    # Draw the current pose
    current_tip_x = (data.pose.pose.pose.position.x - origin.x) * m_to_px
    current_tip_y = (data.pose.pose.pose.position.y - origin.y) * m_to_px
    roll, pitch, yaw = quat_to_rpy(data.pose.pose.pose.orientation)

    arrow_length = 30
    current_foot_x = current_tip_x - (cos(yaw) * arrow_length)
    current_foot_y = current_tip_y - (sin(yaw) * arrow_length)
    cv2.arrowedLine(image, (current_foot_y, current_foot_x), (current_tip_y, current_tip_x), (0, 0, 255), 3)

    # Draw the current visible area
    local_map_metadata_origin_minx = (data.pose.pose.pose.position.x + data.local_obstacle_map.map_metadata.origin.position.x)
    local_map_metadata_origin_miny = (data.pose.pose.pose.position.y + data.local_obstacle_map.map_metadata.origin.position.y)
    local_map_metadata_origin_maxx = ((data.local_obstacle_map.map_metadata.height * m_to_px) * cos(yaw)) + local_map_metadata_origin_minx
    local_map_metadata_origin_maxy = ((data.local_obstacle_map.map_metadata.width * m_to_px) * sin(yaw)) + local_map_metadata_origin_miny

    color = (255, 0, 0)
    thickness = 2
    cv2.line(image, (local_map_metadata_origin_minx, local_map_metadata_origin_miny), (local_map_metadata_origin_minx, local_map_metadata_origin_maxy), color, thickness) 
    cv2.line(image, (local_map_metadata_origin_minx, local_map_metadata_origin_maxy), (local_map_metadata_origin_maxx, local_map_metadata_origin_maxy), color, thickness) 
    cv2.line(image, (local_map_metadata_origin_maxx, local_map_metadata_origin_maxy), (local_map_metadata_origin_maxx, local_map_metadata_origin_miny), color, thickness) 
    cv2.line(image, (local_map_metadata_origin_maxx, local_map_metadata_origin_miny), (local_map_metadata_origin_minx, local_map_metadata_origin_miny), color, thickness) 

    return image

# TODO: is this for right-handed quats?
def quat_to_rpy(quat):
    sinr_cosp = 2 * ( (quat.w*quat.x) + (quat.y*quat.z) )
    cosr_cosp = 1.0 - (2 * ( (quat.x*quat.x) + (quat.y*quat.y) ))
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * ( (quat.w*quat.y) - (quat.z*quat.x) )
    if (abs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * ( (quat.w*quat.z) + (quat.x*quat.y) )
    cosy_cosp = 1.0 - (2 * ((quat.y*quat.y) + (quat.z*quat.z)))
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)

def process_planner_debug_message(data):
    global planner_debug_lock
    global planner_debug_image
    global planner_debug_data
    global last_client_request_time
    global pause_request_time

    if (datetime.datetime.utcnow() - last_client_request_time > pause_request_time):
        return 

    roll, pitch, yaw = quat_to_rpy(data.pose.pose.pose.orientation)

    waypoints = []
    for waypoint in data.path.poses:
        local_point = {}
        local_point['x'] = waypoint.pose.position.x
        local_point['y'] = waypoint.pose.position.y
        local_point['z'] = waypoint.pose.position.z

    local_data = {}
    local_data['debug_image'] = cv_to_byte_stream(draw_planner_debug_image(data))
    local_data['current_x'] = data.pose.pose.pose.position.x
    local_data['current_y'] = data.pose.pose.pose.position.y 
    local_data['current_z'] = data.pose.pose.pose.position.z 
    local_data['current_roll'] = roll
    local_data['current_pitch'] = pitch
    local_data['current_yaw'] = yaw
    local_data['goal_x'] = data.goal.x
    local_data['goal_y'] = data.goal.y
    local_data['goal_z'] = data.goal.z
    local_data['left_motor'] = data.control_signals.left_throttle
    local_data['right_motor'] = data.control_signals.right_throttle
    local_data['waypoints'] = waypoints
    local_data['planner_debug_image'] = cv_to_byte_stream(draw_planner_debug_image(data))

    with planner_debug_lock:
        planner_debug_data = local_data

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
    # TODO: make this also kill the autonomous system

    # Return dummy response to make the framework happy
    return json.dumps({'success':True}), 200, {'ContentType':'application/json'}

@app.route('/data')
def get_cached_topic_data():
    global last_client_request_time
    global planner_debug_lock
    global planner_debug_data

    last_client_request_time = datetime.datetime.utcnow()

    with planner_debug_lock:
        return_data = copy.deepcopy(planner_debug_data)

    return flask.jsonify(return_data)

# Use this to cleanly kill the webpage when ctrl+c is pressed
def sig_handler(sig, frame):
    raise KeyboardInterrupt('ctrl+c pressed')

def main():
    global status_webpage
    global publisher_motor_controller
    global subscriber_debug

    rospy.init_node('control_webpage')

    subscriber_planner_debug = rospy.Subscriber('input_planner_debug', MsgMagellanPlannerDebug, process_planner_debug_message, queue_size=1)
    publisher_motor_controller = rospy.Publisher('output_motor_control', MsgMagellanDrive, queue_size=1)

    signal.signal(signal.SIGINT, sig_handler)

    with open('control_webpage.html', 'r') as f:
        status_webpage = f.read()

    app.run(host='0.0.0.0', port=12345)

if __name__ == '__main__':
    main()
