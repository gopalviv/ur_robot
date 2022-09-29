"""
gopal kumar
"""
# from pygame import joystick
# from pyjoystick.sdl2 import Key, Joystick, run_event_loop
import math
import sys
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

# The Joint position the robot starts at

robot1 = rtde_control.RTDEControlInterface("192.168.137.3")
robo_io = rtde_io.RTDEIOInterface("192.168.137.3")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.137.3")
# robo_actual_pose = rtde_r.getActualTCPPose()



"""FUNCTIONS _____________________________________________________________________________"""
# initialise robot with URBasic
print("initialising robot")

robotModel = URBasic.robotModel.RobotModel()
# dc = DepthCamera()

robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1)  # just a short wait to make sure everything is initialised








# robot.set_payload(0, (0, 0, 0))


class joint_stop(Thread):
    def __init__(self):
        super().__init__()
        self.ACCELERATION = 0
        self.VELOCITY = 0

    # def run(self, key):
    def on_release(self, key):
        print('{0} release'.format(key))
        if key == key.esc:
            print("hiiiiiiiiiiii")
            # print("gggggggggggg")
            self.ACCELERATION = 0
            self.VELOCITY = 0
            robot.stopj(a=0)
            return self, key


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

point = (400, 300)

# Initialize Camera Intel Realsense..............................................................................
dc = DepthCamera()


# Create mouse event


def show_distance(event, x, y, args, params):
    global point
    point = (x, y)


cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

# while False:
#     ret, depth_frame, color_frame = dc.get_frame()
#
#     # Show distance for a specific point
#     cv2.circle(color_frame, point, 4, (0, 0, 255))  # here 4 is radius of circle (o,o,255) is for colour
#     distance = depth_frame[point[1], point[0]]  # here point[1] means 300 point[0] means 400
#     # if key==27:
#     #     print(distance)
#
#     cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
#                 2)  # here -20 is to put the text above the point
#
#     cv2.imshow("depth frame", depth_frame)  # here colour frame
#     cv2.imshow("Color frame", color_frame)  # here depth frame ,
#     key = cv2.waitKey(1)  # withkey(0) is freezes the frame, withkey(1) waits 1ms and moves to the next frame
#     if key == 27:  # escape key
#         print(distance)
#
# """" Note:- depth frame is that frame which has the same shape of a colour frame but instead of containing
#     the image with color
#     so the original frame from the camera this arrays is containing  for each pixel
#     the distance of on the space from the camera to the object
#     to know how far is object from camera in order to get these information we need to refer to
#     the depth frame """


class camera_thead(Thread):
    def __init__(self):
        super().__init__()
        self.distance = 0

    def run(self):

        while True:
            ret, depth_frame, color_frame = dc.get_frame()
            # Show distance for a specific point
            cv2.circle(color_frame, point, 4, (0, 0, 255))  # here 4 is radius of circle (o,o,255) is for colour
            self.distance = depth_frame[point[1], point[0]]  # here point[1] means 300 point[0] means 400
            # if key==27:
            #     print(distance)

            cv2.putText(color_frame, "{}mm".format(self.distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2,
                        (0, 0, 0),
                        2)  # here -20 is to put the text above the point

            cv2.imshow("depth frame", depth_frame)  # here colour frame
            cv2.imshow("Color frame", color_frame)  # here depth frame ,
            key = cv2.waitKey(1)  # withkey(0) is freezes the frame, withkey(1) waits 1ms and moves to the next frame
            if key == 27:  # escape key
                break

            return self


camera = camera_thead()


class joint_angle_thread(Thread):
    def __init__(self):
        super().__init__()
        self.angle_base = 0
        self.angle_shoulder = 0
        self.angle_elbow = 0
        self.angle_wrist1 = 0
        self.angle_wrist2 = 0
        self.angle_wrist3 = 0
        self.distance = 0

    def run(self):
        while True:
            print("joint thread run")
            # time.sleep(1)
            robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)

            qnear = robot.get_actual_joint_positions()
            robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
            # robot.reset_error()
            print("robot initialised")

            time.sleep(1)  # just a short wait to make sure everything is initialised
            self.angle_base = int(math.degrees(qnear[0]))
            self.angle_shoulder = int(math.degrees(qnear[1]))
            self.angle_elbow = int(math.degrees(qnear[2]))
            self.angle_wrist1 = int(math.degrees(qnear[3]))
            self.angle_wrist2 = int(math.degrees(qnear[4]))
            self.angle_wrist3 = int(math.degrees(qnear[5]))
            robot.reset_error()

            return self


t0 = joint_angle_thread()
g = 0
n = 0


def on_press(key):
    print('{0} pressed'.format(
        key))

    t0 = joint_angle_thread()

    global g

    # t0.start()
    # t0.join()
    # # VELOCITY = 2
    # ACCELERATION=2
    print("gopal")

    robo_actual_pose = rtde_r.getActualTCPPose()
    # wrist2 = robo_actual_pose[4]  # g = t0.angle_wrist2
    # shoulder = robo_actual_pose[1]
    base = robo_actual_pose[0]

    if key == key.up:
        g += 1
        tme = g
        # t0.start()
        # t0.join()

        desired_pose = [math.radians(t0.angle_base + tme),
                        math.radians(t0.angle_shoulder),  # shoulder
                        math.radians(t0.angle_elbow),  # elbow
                        math.radians(t0.angle_wrist1),  # wrist1,
                        math.radians(t0.angle_wrist2),  # wrist2
                        math.radians(t0.angle_wrist3)]
        # robot.speedj(qd=math.radians(2), a=5, t=0.1)
        robot.servoj(q=desired_pose, lookahead_time=10, t=1, gain=100)

    elif key == key.down:
        # g = 360 - t0.angle_wrist2
        g -= 1
        tme = g
        desired_pose = [math.radians(t0.angle_base + tme),
                        math.radians(t0.angle_shoulder),  # shoulder
                        math.radians(t0.angle_elbow),  # elbow
                        math.radians(t0.angle_wrist1),  # wrist1,
                        math.radians(t0.angle_wrist2),  # wrist2
                        math.radians(t0.angle_wrist3)]
        robot.servoj(q=desired_pose, lookahead_time=10, t=1, gain=100)

    elif key == key.left:
        # g = 360 - t0.angle_wrist2
        g -= 1
        tme = g
        desired_pose = [math.radians(robo_actual_pose[0]),
                        math.radians(t0.angle_shoulder),  # shoulder
                        math.radians(t0.angle_elbow),  # elbow
                        math.radians(t0.angle_wrist1),  # wrist1,
                        math.radians(t0.angle_wrist2),  # wrist2
                        math.radians(t0.angle_wrist3 + tme)]
        robot.servoj(q=desired_pose, lookahead_time=10, t=1, gain=100)

    elif key == key.right:
        g = 0
        tme = g
        desired_pose = [math.radians(robo_actual_pose[0]),
                        math.radians(t0.angle_shoulder),  # shoulder
                        math.radians(t0.angle_elbow),  # elbow
                        math.radians(t0.angle_wrist1),  # wrist1,
                        math.radians(t0.angle_wrist2),  # wrist2
                        math.radians(t0.angle_wrist3 + tme)]
        robot.servoj(q=desired_pose, lookahead_time=10, t=1, gain=100)

    # elif key == key.enter:
    #     robot.stopl(a=0)
    # elif key==key.esp:
    #
    #     g = camera.distance
    #     print(c)
    #     g += 1
    #     tme = g
    #
    #     desired_pose = [math.radians(t0.angle_base),
    #                     math.radians(t0.angle_shoulder),  # shoulder
    #                     math.radians(t0.angle_elbow),  # elbow
    #                     math.radians(t0.angle_wrist1),  # wrist1,
    #                     math.radians(t0.angle_wrist2),  # wrist2
    #                     math.radians(t0.angle_wrist3 + tme)]
    #     robot.servoj(q=desired_pose, lookahead_time=10, t=1, gain=100)


with Listener(on_press=on_press) as listener:
    sleep(0.1)
    listener.join()

if __name__ == "__main__":
    t1 = threading.Thread(target=joint_angle_thread, name='t1')
    c = threading.Thread(target=camera_thead, name='c')

    t1.start()
    c.start()

    t1.join()
    c.join(0.2)
#
