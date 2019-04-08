import random
import cv2
import h5py 
import numpy as np
import pandas as pd
import json
import os
import sys

from keras.models import Sequential, Model, load_model

import detector_utils

import tensorflow as tflow

class ConeDetector(object):
    def __init__(self, model_path, opening_iterations, debug):
        self.model = load_model(model_path, custom_objects={'raw_jaccard_metric': detector_utils.raw_jaccard_metric})
        first_layer = self.model.get_layer(index=0)
        self.first_layer_shape = (1, first_layer.batch_input_shape[1], first_layer.batch_input_shape[2], first_layer.batch_input_shape[3])
        self.graph = tflow.get_default_graph()
        self.prediction_buffer = np.zeros(self.first_layer_shape)
        self.opening_iterations = opening_iterations
        self.debug = debug

    # returns centroid
    def predict_image(self, image):
        if (image.shape[0] != self.prediction_buffer.shape[1] or image.shape[1] != self.prediction_buffer.shape[2]):
            self.prediction_buffer[0, :, :, :] = cv2.resize(image, (self.prediction_buffer.shape[2], self.prediction_buffer.shape[1]), cv2.INTER_LANCZOS4)
        else:
            self.prediction_buffer[0, :, :, :] = image

        with self.graph.as_default():

            output_image = self.model.predict(self.prediction_buffer)[0]

        output_image = (output_image > 0.5).astype(np.uint8)

        processed_output_image = self.post_process_model_output(output_image, self.opening_iterations)
        centroid_y, centroid_x, outline = self.find_centroid(processed_output_image)

        centroid = (centroid_x, centroid_y)

        if not self.debug:
            return point, None, None, None

        draw_debug_img = image.copy()

        if (outline is not None):
            draw_debug_img = cv2.polylines(draw_debug_img, outline, True, (255, 0, 255), 4, cv2.LINE_AA)

        if (centroid is not None):
            draw_debug_img = cv2.circle(draw_debug_img, (centroid[1], centroid[0]), 5, (0, 255, 255), -1, cv2.LINE_AA)

        return centroid, image, detector_utils.make_mask_image(output_image), detector_utils.make_mask_image(processed_output_image), draw_debug_img

    def post_process_model_output(self, prediction_image, iterations):

        if iterations is not None and iterations > 0:
            kernel = np.ones((5,5), np.uint8)
            return cv2.morphologyEx(prediction_image, cv2.MORPH_OPEN, kernel, iterations=iterations)

        return prediction_image

    def get_shape(self):
        return (self.first_layer_shape[2], self.first_layer_shape[1])

    def find_centroid(self, prediction_image):
        out_image = prediction_image.copy()
        contours, hierarchy = cv2.findContours(prediction_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        biggest_contour = None
        biggest_contour_area = None
        if (hierarchy is not None):
            index = 0
            while (index >= 0):
                area = cv2.contourArea(contours[index])
                if biggest_contour_area is None or biggest_contour_area < area:
                    biggest_contour = contours[index]
                    biggest_contour_area = area
                index = hierarchy[0][index][0]

        if biggest_contour is None:
            return None, None, None

        moments = cv2.moments(biggest_contour)
        centroid_x = int(moments['m01'] / moments['m00'])
        centroid_y = int(moments['m10'] / moments['m00'])
        
        return centroid_y, centroid_x, biggest_contour

    def is_debug(self):
        return self.debug



