import cv2
import numpy as np
import pandas as pd
import os
import sys
import detector
import matplotlib.pyplot as plt

def parse_args_cmd_line():
    parsed_args = {}

    for i in range(1, len(sys.argv), 2):
        if sys.argv[i] == '-m':
            parsed_args['model_file'] = sys.argv[i+1]
        elif sys.argv[i] == '-o':
            parsed_args['output_file'] = sys.argv[i+1]
        elif sys.argv[i] == '-i':
            parsed_args['image_path'] = sys.argv[i+1]
        elif sys.argv[i] == '-q':
            parsed_args['opening_iterations'] = int(sys.argv[i+1])

    return parsed_args

def print_usage():
    print('Usage: detector_test_console.py {args}')
    print('arguments:')
    print('\t-m: model file')
    print('\t-o: output file')
    print('\t-i: input image file')
    print('\t-q: opening iterations')

def main():
    parsed_args = parse_args_cmd_line()
    if (len(parsed_args) != 4):
        print_usage()
        return

    print('Running detection with the following parameters:')
    print('\tmodel: {0}'.format(parsed_args['model_file']))
    print('\tinput image: {0}'.format(parsed_args['image_path']))
    print('\toutput image: {0}'.format(parsed_args['output_file']))

    print('Initializing detector...')
    cone_detector = detector.ConeDetector(parsed_args['model_file'], parsed_args['opening_iterations'], True)

    print('Loading image')
    read_image = cv2.imread(parsed_args['image_path'])[...,::-1]

    resize_params = cone_detector.get_shape()

    print('Resizing image to {0}x{1}'.format(resize_params[1], resize_params[0]))
    read_image = cv2.resize(read_image, resize_params, cv2.INTER_LANCZOS4)

    print('image shape: {0}'.format(read_image.shape))

    print('Predicting...')
    centroid, input_image, output_image, processed_output_image, debug_img = cone_detector.predict_image(read_image)

    print('Generating debug image...')
    fig, ax = plt.subplots(nrows=2, ncols=2, figsize=(10,10))

    ax[0][0].imshow(input_image)
    ax[0][0].set_title('Original Image')

    ax[1][0].imshow(output_image)
    ax[1][0].set_title('Raw Output Image')

    ax[1][1].imshow(processed_output_image)
    ax[1][1].set_title('Processed Output Image')

    ax[0][1].imshow(debug_img)
    ax[0][1].set_title('Debug image')

    print('Saving to {0}...'.format(parsed_args['output_file']))
    plt.savefig(parsed_args['output_file'])

    print('Graceful termination.')

if __name__ == '__main__':
    main()
