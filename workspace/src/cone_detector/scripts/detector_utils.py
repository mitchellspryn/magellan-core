import numpy as np
import pandas as pd
import sys
import keras.backend as K

def parse_bool(val):
    val = val.lower()
    if (val == 'true' or val == 't'):
        return True
    elif (val == 'false' or val == 'f'):
        return False

    error_msg = 'Unable to convert "{0}" to boolean.'.format(val)
    rospy.logfatal(error_msg)
    raise ValueError(error_msg)

def parse_int(val):
    try:
        i = int(val)
        return i
    except ValueError:
        error_msg = 'Unable to convert "{0}" to int'.format(val)
        rospy.logfatal(error_msg)
        raise ValueError(error_msg)

def parse_command_line_args(args):
    cmd_line_args = {}
    cmd_line_args['debug'] = False
    cmd_line_args['verbose_print'] = False

    for i in range(1, len(args), 2):
        if args[i] == '-m':
            cmd_line_args['model_path'] = args[i+1]
        elif args[i] == '-d':
            cmd_line_args['debug'] = parse_bool(args[i+1])
        elif args[i] == '-o':
            cmd_line_args['openiniterations'] = parse_int(args[i+1])

    return cmd_line_args

def smoothed_jaccard(y_true, y_pred, smooth=100):
    intersection = K.sum(K.sum(K.sum(K.abs(y_true * y_pred), axis=1), axis=1), axis=1)

    sum_ = K.sum(K.sum(K.sum(K.abs(y_true) + K.abs(y_pred), axis=1), axis=1), axis=1)
    
    jac = (intersection + smooth) / (sum_ - intersection + smooth)
    
    # Create a tensor of all ones (sum_ has to be positive)
    # Cannot use K.ones() without re-initializing on every call
    # Also cannot initialize once globally and index with K.gather() 
    #    because during the compile phase, y_true.shape[0] is None
    local_ones = sum_ + 1
    local_ones = local_ones / local_ones
    
    jac_filt = K.switch(K.equal(jac, jac), jac, local_ones)
    
    return jac_filt

def raw_jaccard_metric(y_true, y_pred):
    return smoothed_jaccard(y_true, y_pred, smooth=0)

def np_jaccard(y_true, y_pred):
    intersect = np.sum(y_true * y_pred)
    union = np.sum(np.abs(y_true)) + np.sum(np.abs(y_pred)) - intersect
    
    if (union == 0):
        return 1.0
    
    return intersect / union


def make_mask_image(mask):
    mask_img = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    
    for y in range(0, mask.shape[0], 1):
        for x in range(0, mask.shape[1], 1):
            if (mask[y, x] >= 0.5):
                mask_img[y, x, :] = [255, 255, 255]
    return mask_img

