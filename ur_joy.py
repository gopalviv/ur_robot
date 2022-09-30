"""
gopal kumar
"""
# from pygame import joystick
# from pyjoystick.sdl2 import Key, Joystick, run_event_loop
import math
import sys
import multiprocessing
import threading
import time
from threading import *
from time import sleep
import cv2
from pynput.keyboard import Listener
import URBasic
import rtde_io
import rtde_control
import rtde_receive

global robot
global robo_io
global rtde_r

"""""""""""""""""SETTINGS AND VARIABLES ________________________________________________________________"""

RASPBERRY_BOOL = False
# If this is run on a linux system, a picamera will be used.
# If you are using a linux system, with a webcam instead of a raspberry pi delete the following if-statement
if sys.platform == "linux":
    RASPBERRY_BOOL = True

ROBOT_IP = '192.168.137.3'
# ROBOT_IP = '192.168.88.128'

# playsound('audio.mp3')

ACCELERATION = 0  # Robot acceleration value
VELOCITY = 0  # Robot speed value
a = 0
b = 1
base_p = 0
wrist2_p = 0

# The Joint position the robot starts at
#
# robot1 = rtde_control.RTDEControlInterface("192.168.137.3")
# # robo_io = rtde_io.RTDEIOInterface("192.168.137.3")
# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.137.3")
# robo_actual_pose = 0

"""FUNCTIONS _____________________________________________________________________________"""
# initialise robot with URBasic
print("initialising robot")

robotModel = URBasic.robotModel.RobotModel()
# dc = DepthCamera()

robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1)  # just a short wait to make sure everything is initialised

# robot.set_payload(0, (0, 0, 0))

qnear = robot.get_actual_joint_positions()
angle_base = int(math.degrees(qnear[0]))
angle_shoulder = int(math.degrees(qnear[1]))
angle_elbow = int(math.degrees(qnear[2]))
angle_wrist1 = int(math.degrees(qnear[3]))
angle_wrist2 = int(math.degrees(qnear[4]))
angle_wrist3 = int(math.degrees(qnear[5]))
robot_startposition = (math.radians(0),
                       math.radians(0),  # shoulder
                       math.radians(0),  # elbow
                       math.radians(0),  # wrist1,
                       math.radians(0),  # wrist2
                       math.radians(0))
robot.movej(q=robot_startposition, a=2, v=2)

robot.reset_error()
print("robot initialised")
time.sleep(1)
y = 0

""""""""""'""""""""""""""""""""...................realsense camera code........................................."""

from realsense_depth import *

class Camera_thead(Thread):
    def __init__(self):
        super().__init__()
        self.distance = 0
        self.dc = DepthCamera()
        self.point = (400, 300)

    def show_distance(self, event, x, y, args, params):
        self.point = (x, y)

    def run(self):

        # Initialize Camera Intel Realsense..............................................................................

        cv2.namedWindow("Color frame")
        cv2.setMouseCallback("Color frame", self.show_distance)
        while True:
            ret, depth_frame, color_frame = self.dc.get_frame()
            # Show distance for a specific point
            cv2.circle(color_frame, self.point, 4, (0, 0, 255))  # here 4 is radius of circle (o,o,255) is for colour
            self.distance = depth_frame[self.point[1], self.point[0]]  # here point[1] means 300 point[0] means 400

            cv2.putText(color_frame, "{}mm".format(self.distance), (self.point[0], self.point[1] - 20),
                        cv2.FONT_HERSHEY_PLAIN, 2,
                        (0, 0, 0),
                        2)  # here -20 is to put the text above the point

            cv2.imshow("depth frame", depth_frame)  # here colour frame
            cv2.imshow("Color frame", color_frame)  # here depth frame ,
            key = cv2.waitKey(2)  # withkey(0) is freezes the frame, withkey(1) waits 1ms and moves to the next frame

            if key == 27:
                break


"""" Note:- depth frame is that frame which has the same shape of a colour frame but instead of containing                   
    the image with color                                                                                                     
    so the original frame from the camera this arrays is containing  for each pixel                                          
    the distance of on the space from the camera to the object                                                               
    to know how far is object from camera in order to get these information we need to refer to                              
    the depth frame """

camera = Camera_thead()
degree = camera.distance * 0.001  # 1 mm = 0.001 deg.
print(degree)

def on_press(key):
    global wrist2_p
    global base_p
    if key == key.up:
        base_p += 2
    elif key == key.down:
        base_p -= 2
    elif key == key.left:
        wrist2_p += 2
    elif key == key.right:
        wrist2_p -= 2

    elif key == key.space:
        wrist2_p += 2
    bas = angle_base + base_p
    wr = wrist2_p + angle_wrist2
    print(bas)
    desired_pose = [math.radians(bas),
                    math.radians(angle_shoulder),  # shoulder
                    math.radians(angle_elbow),  # elbow
                    math.radians(angle_wrist1),  # wrist1,
                    math.radians(angle_wrist2),  # wrist2
                    math.radians(wr)]
    robot.servoj(q=desired_pose, lookahead_time=10, t=1, gain=100)


camera.start()

with Listener(on_press=on_press) as listener:
    sleep(0.1)
    listener.join()
