#!/usr/bin/env python3
import flask
import os
import rospy
import signal
import warnings
import cv2
import threading
import base64

# Upon termination, bottle seems to complain about a resource warning
# It doesn't seem to affect anything, so ignore it
#warnings.simplefilter("ignore", ResourceWarning)

status_webpage = None
dummy_image = None
dummy_background_thread = None
app = flask.Flask(__name__)

@app.route('/')
def control():
    global status_webpage
    return status_webpage

@app.route('/static/<filename>')
def static_file(filename):
    print(filename)
    return flask.send_from_directory(os.getcwd(), filename)

def background_dummy_thread():
    global dummy_image
    video_capture = cv2.VideoCapture(0)
    while(True):
        ret, frame = video_capture.read()
        ret, jpg = cv2.imencode('.jpeg', frame)
        dummy_image = base64.b64encode(jpg).decode('utf-8')
        #dummy_image = jpg.tobytes()

def dummy_image_generator():
    global dummy_image
    while(True):
        try:
            if (dummy_image is not None):
                yield(b'--frame\r\nContent-Type:image/jpeg\r\n\r\n' + dummy_image + b'\r\n\r\n')
        except Exception as e:
            print(str(e))

def get_single_dummy_image():
    if (dummy_image is not None):
        return flask.jsonify({'data': 'data:image/jpeg;charset=utf-8;base64,' + dummy_image})
    return flask.jsonify({'data': 'data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg=="'})

@app.route('/bottom-camera-image')
def bottom_camera_image():
    #return flask.Response(dummy_image_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return get_single_dummy_image()

@app.route('/xtion-rgb-image')
def xtion_rgb_image():
    #return flask.Response(dummy_image_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return get_single_dummy_image()

@app.route('/xtion-depth-image')
def xtion_depth_image():
    #return flask.Response(dummy_image_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return get_single_dummy_image()

@app.route('/lidar-image')
def lidar_image():
    #return flask.Response(dummy_image_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return get_single_dummy_image()

@app.route('/dummy-image')
def image():
    #return flask.Response(dummy_image_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return get_single_dummy_image()

# Use this to cleanly kill the webpage when ctrl+c is pressed
def sig_handler(sig, frame):
    raise KeyboardInterrupt('ctrl+c pressed')

def main():
    global status_webpage
    global video_capture
    #TODO: ros stuff
    rospy.init_node('control_webpage')

    dummy_background_thread = threading.Thread(target=background_dummy_thread, args=())
    dummy_background_thread.daemon = True
    dummy_background_thread.start()

    signal.signal(signal.SIGINT, sig_handler)

    with open('control_webpage.html', 'r') as f:
        status_webpage = f.read()

    app.run(host='0.0.0.0')

if __name__ == '__main__':
    main()
