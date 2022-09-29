"""
gopal
"""

import cv2
import pyrealsense2
import realsense_depth

point = (400, 300)


def show_distance(event, x, y, args, params):
    global point
    point = (x, y)


# Initialize Camera Intel Realsense..............................................................
dc = realsense_depth.DepthCamera()

# Create mouse event
cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()

    # Show distance for a specific point
    cv2.circle(color_frame, point, 4, (0, 0, 255))  # here 4 is radius of circle (o,o,255) is for colour
    distance = depth_frame[point[1], point[0]]  # here point[1] means 300 point[0] means 400
    # if key==27:
    #     print(distance)

    cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                2)  # here -20 is to put the text above the point

    cv2.imshow("depth frame", depth_frame)  # here colour frame
    cv2.imshow("Color frame", color_frame)  # here depth frame ,
    key = cv2.waitKey(1)  # withkey(0) is freezes the frame, withkey(1) waits 1ms and moves to the next frame
    if key == 27:  # escape key
        print(distance)


"""" Note:- depth frame is that frame which has the same shape of a colour frame but instead of containing 
    the image with color  
    so the original frame from the camera this arrays is containing  for each pixel 
    the distance of on the space from the camera to the object 
    to know how far is object from camera in order to get these information we need to refer to 
    the depth frame """
