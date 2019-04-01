import random
import rospy
import cv2
import h5py 
import numpy as np
import pandas as pd
import json
import os
import sys

from cv_bridge import CvBridge, CvBridgeError


def run_node():
    rospy.init_node('cone_detector')
    rospy.loginfo('Cone detector started.')

    # TODO: Write logic for actually using the model

    rospy.spin()

if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass

