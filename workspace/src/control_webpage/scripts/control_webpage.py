#!/usr/bin/env python3
import flask
import os
import rospy
import signal
import warnings
import cv2

# Upon termination, bottle seems to complain about a resource warning
# It doesn't seem to affect anything, so ignore it
#warnings.simplefilter("ignore", ResourceWarning)

status_webpage = None
video_capture = None
app = flask.Flask(__name__)

@app.route('/')
def control():
    global status_webpage
    return status_webpage

@app.route('/static/<filename>')
def static_file(filename):
    print(filename)
    return flask.send_from_directory(os.getcwd(), filename)

def image_generator():
    global video_capture
    while(True):
        try:
            ret, frame = video_capture.read()
            ret, jpg = cv2.imencode('.jpg', frame)
            jpg_bytes = jpg.tobytes()
            yield(b'--frame\r\nContent-Type:image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n\r\n')
        except Exception as e:
            print(str(e))


@app.route('/image')
def image():
    return flask.Response(image_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Use this to cleanly kill the webpage when ctrl+c is pressed
def sig_handler(sig, frame):
    raise KeyboardInterrupt('ctrl+c pressed')

def main():
    global status_webpage
    global video_capture
    #TODO: ros stuff
    rospy.init_node('control_webpage')

    video_capture = cv2.VideoCapture(0)
    signal.signal(signal.SIGINT, sig_handler)

    with open('control_webpage.html', 'r') as f:
        status_webpage = f.read()

    app.run(host='0.0.0.0')

if __name__ == '__main__':
    main()
