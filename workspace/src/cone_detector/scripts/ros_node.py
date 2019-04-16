#!/usr/bin/env python3
import sys

import random
import rospy
import cv2
import h5py 
import numpy as np
import pandas as pd
import json
import os
import threading

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32

from keras.models import Sequential, Model, load_model

from magellan_messages.msg import MsgConeDetectorOutput
from sensor_msgs.msg import Image

import detector
import detector_utils

import tensorflow as tflow

g_cv_bridge = CvBridge()
g_cone_detector = None
g_lock = threading.Lock()
g_input_topic = None
g_output_topic = None
g_debug_topics = None

def predict_callback(data):
    global g_cone_detector
    global g_input_topic
    global g_output_topic
    global g_debug_topics
    global g_lock
    global g_cv_bridge

    with g_lock:
        if g_cone_detector is None:
            rospy.logfatal('Cone detector is None.')
            return
        
        img = g_cv_bridge.imgmsg_to_cv2(data, 'bgr8')[...,::-1]
        centroid, input_image, raw_output, processed_output, debug_draw = g_cone_detector.predict_image(cv2.resize(img, g_cone_detector.get_shape(), cv2.INTER_LANCZOS4))

        point = Point32()
        
        if (centroid is None or centroid[0] is None or centroid[1] is None):
            point.x = -1
            point.y = -1
        else:
            point.x = centroid[1]
            point.y = centroid[0]

        point.z = 0

        if (g_output_topic is not None):
            msg = MsgConeDetectorOutput()
            msg.header.stamp = data.header.stamp
            msg.detected_cones = [point]

            g_output_topic.publish(msg)

        if (g_cone_detector.is_debug()):
            g_debug_topics[0].publish(g_cv_bridge.cv2_to_imgmsg(input_image[...,::-1], 'bgr8'))
            g_debug_topics[1].publish(g_cv_bridge.cv2_to_imgmsg(raw_output[...,::-1], 'bgr8'))
            g_debug_topics[2].publish(g_cv_bridge.cv2_to_imgmsg(processed_output[...,::-1], 'bgr8'))
            g_debug_topics[3].publish(g_cv_bridge.cv2_to_imgmsg(debug_draw[...,::-1], 'bgr8'))


def init_detector_and_topics(parsed_args):
    global g_cone_detector
    global g_input_topic
    global g_output_topic
    global g_debug_topics

    if 'model_path' not in parsed_args:
        rospy.logerror('Model not specified. Please specify the proper model with -m')
        return False

    if 'opening_iterations' not in parsed_args:
        parsed_args['opening_iterations'] = 0

    if 'debug' not in parsed_args:
        parsed_args['debug'] = False

    g_cone_detector = detector.ConeDetector(parsed_args['model_path'], parsed_args['opening_iterations'], parsed_args['debug'])

    g_input_topic = rospy.Subscriber('input_topic', Image, predict_callback, queue_size=1)
    g_output_topic = rospy.Publisher('output_topic', MsgConeDetectorOutput, queue_size=1000)
    
    if parsed_args['debug']:
        g_debug_topics = []
        g_debug_topics.append(rospy.Publisher('debug_original_image_topic', Image, queue_size=1000))
        g_debug_topics.append(rospy.Publisher('debug_predicted_image_topic', Image, queue_size=1000))
        g_debug_topics.append(rospy.Publisher('debug_processed_image_topic', Image, queue_size=1000))
        g_debug_topics.append(rospy.Publisher('debug_draw_image_topic', Image, queue_size=1000))

    return True


def run_node():
    rospy.init_node('cone_detector')

    parsed_args = detector_utils.parse_command_line_args(sys.argv)
    if not init_detector_and_topics(parsed_args):
        rospy.logfatal('Could not initialize detector.')
        raise ValueError #TODO: better error type?

    rospy.loginfo('Cone detector started.')
    rospy.spin()

if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass

