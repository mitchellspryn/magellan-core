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

from magellan_msgs.msg.cone_detector import MsgConeDetectorDebug
from magellan_msgs.msg.cone_detector import MsgConeDetectorOutput

import detector
import detector_utils

import tensorflow as tflow

g_cv_bridge = CvBridge()
g_cone_detector = None
g_lock = threading.Lock()
g_input_topic = None
g_output_topic = None
g_debug_topic = None

def predict_callback(data):
    global g_cone_detector
    global g_input_topic
    global g_output_topic
    global g_debug_topic
    global g_lock
    global g_cv_bridge

    with g_lock:
        if g_cone_detector is None:
            rospy.logfatal('Cone detector is None.')
            return
        
        centroid, input_image, raw_output, processed_output, debug_draw = g_cone_detector.predict_image(g_cv_bridge.imgmsg_to_cv2(data.image, 'bgr8'))

        if (g_output_topic is not None):
            msg = MsgConeDetectorOutput()
            msg.header.stamp = data.header.stamp
            msg.detected_cones = [point]

            g_output_topic.publish(msg)

        if (g_cone_detector.is_debug()):
            msg = MsgConeDetectorDebug()
            msg.header.stamp = data.header.stamp
            msg.input_frame = input_image
            msg.raw_prediction = raw_output
            msg.processed_prediction = processed_output
            msg.debug_image = debug_draw

            g_debug_topic.publish(msg)


def init_detector_and_topics(parsed_args):
    global g_cone_detector
    global g_input_topic
    global g_output_topic
    global g_debug_topic

    if 'model_path' not in parsed_args:
        rospy.logerror('Model not specified. Please specify the proper model with -m')
        return False

    if 'opening_iterations' not in parsed_args:
        parsed_args['opening_iterations'] = 0

    if 'debug' not in parsed_args:
        parsed_args['debug'] = False

    g_cone_detector = detector.ConeDetector(parsed_args['model_path'], parsed_args['opening_iterations'], parsed_args['debug'])

    g_input_topic = rospy.Subscriber('input_topic', msg_type, predict_callback, queue_size=1)
    g_output_topic = rospy.Publisher('output_topic', msg_type, queue_size=1000)
    
    if parsed_args['debug']:
        g_debug_topic = rospy.Publisher('debug_output_topic', msg_type, queue_size=1000)


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

