#!/usr/bin/python
import io
import picamera
import cv2
import numpy as np
import sys
import math
import serial
import time
import RPi.GPIO as GPIO
from multiprocessing.dummy import Pool as ThreadPool

# control libraries
from MoveForward import MoveForward
from Rotate import *
from SlowStepForward import *
from LegsControl import *

# camera variables
RESOLUTION_X = 320
RESOLUTION_Y = 240

# image preprocessing variables
THRESH_VALUE1 = 60
THRESH_VALUE2 = 10

# reference point for scanning
SCAN_RADIUS = RESOLUTION_X / 4
SCAN_HEIGHT = RESOLUTION_Y - 10
SCAN_POS_X = RESOLUTION_X / 2

SCAN_RADIUS_REG = 50  # scan radius from above point
NUMBER_OF_CIRCLES = 3  # number of iterations


def scanLine(image, point, radius):
    x = point[0];
    y = point[1];

    scan_start = x - radius
    scan_end = x + radius
    row = image[y]
    data = np.empty(radius * 2)
    data[:] = row[scan_start:scan_end]

    return data;


def coordinateFromPoint(origin, angle, radius):
    xo = origin[0]
    yo = origin[1]

    # Work out the co-ordinate for the pixel on the circumference of the circle
    x = xo - radius * math.cos(math.radians(angle))
    y = yo + radius * math.sin(math.radians(angle))

    # We only want whole numbers
    x = int(round(x))
    y = int(round(y))
    return (x, y);


def scanCircle(image, point, radius, look_angle):
    x = point[0];
    y = point[1];
    scan_start = x - radius
    scan_end = x + radius

    endpoint_left = coordinateFromPoint(point, look_angle - 90, radius)
    endpoint_right = coordinateFromPoint(point, look_angle + 90, radius)

    # We are only going to scan half the circumference
    data = np.zeros(shape=(180, 3))

    # Getting the co-ordinates and value for every degree in the semi circle
    startAngle = look_angle - 90

    returnVal = True
    for i in range(0, 180, 1):
        current_angle = startAngle + i
        scan_point = coordinateFromPoint(point, current_angle, radius)

        if inImageBounds(image, scan_point[0], scan_point[1]):
            imageValue = image[scan_point[1]][scan_point[0]]
            data[i] = [imageValue, scan_point[0], scan_point[1]]
        else:
            returnVal = False
            break;

    return returnVal, data;


def findInCircle(scan_data):
    data = np.zeros(shape=(len(scan_data) - 1, 1))
    data[0] = 0
    data[len(data) - 1] = 0
    for index in range(1, len(data)):
        data[index] = scan_data[index - 1][0] - scan_data[index][0]

    # left and right should be the boundry values.
    # first element will be the image value
    # second element will be the index of the data item
    left = [0, 0]
    right = [0, 0]

    for index in range(0, len(data)):
        if data[index] > left[1]:
            left[1] = data[index]
            left[0] = index

        if data[index] < right[1]:
            right[1] = data[index]
            right[0] = index

    leftx = int(scan_data[left[0]][1])
    lefty = int(scan_data[left[0]][2])
    lefti = left[0]
    rightx = int(scan_data[right[0]][1])
    righty = int(scan_data[right[0]][2])
    righti = right[0]

    centre_index = int(round((righti + lefti) / 2))

    position = [int(scan_data[centre_index][1]), int(scan_data[centre_index][2])]

    # # mid point, where we believe is the centre of the line
    # cv2.circle(display_image, (position[0], position[1]), 5, (255, 255, 255), -1, 8, 0)
    # # left boundrary dot on the line
    # cv2.circle(display_image, (leftx, lefty), 2, (0, 0, 102), 2, 8, 0)
    # # right boundrary dot on the line
    # cv2.circle(display_image, (rightx, righty), 2, (0, 0, 102), 2, 8, 0)

    return position;


def inImageBounds(image, x, y):
    return x >= 0 and y >= 0 and y < len(image) and x < len(image[y])


def findLine(scan_data, x, y, radius):
    data = np.empty(len(scan_data) - 1)
    data[0] = 0
    data[len(data) - 1] = 0
    for index in range(1, len(data)):
        data[index] = scan_data[index - 1] - scan_data[index]

    scan_start = x - radius
    scan_end = x + radius

    left = [0, 0]
    right = [0, 0]

    for index in range(0, len(data)):
        if data[index] > left[1]:   
            left[1] = data[index]
            left[0] = index

        if data[index] < right[1]:
            right[1] = data[index]
            right[0] = index

    line_position = (right[0] + left[0]) / 2
    return (scan_start + line_position, y);


def lineAngle(point1, point2):
    angle = round(math.atan2((point2[1] - point1[1]), -(point2[0] - point1[0])) * 180 / math.pi)
    return angle


def lineLength(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    return int(round(math.sqrt(dx * dx + dy * dy)));


def main():
    ########## Robot initialize ########

    rear_left = RobotLeg(6, 5, m2_revert=2, servoMinOffset=-0.05)
    rear_right = RobotLeg(10, 9, m2_revert=1, m1_revert=3, servoMaxOffset=-0.05, servoMinOffset=-0.25)
    front_left = RobotLeg(3, 2, m2_revert=1, m1_revert=3, servoMinOffset=-0.15)
    front_right = RobotLeg(11, 12, m2_revert=2, servoMinOffset=-0.05)

    move_forward = MoveForward(front_left, front_right, rear_left, rear_right)

    KeepAliveThread()

    rear_left.move_h1(MIN_h1_rear)
    rear_right.move_h1(MIN_h1_rear)
    front_left.move_h1(MAX_h1_front)
    front_right.move_h1(MAX_h1_front)

    for x in range(-90, MAX_h2_front):
        rear_left.move_h2(x * MAX_h2_rear / MAX_h2_front)
        rear_right.move_h2(x * MAX_h2_rear / MAX_h2_front)
        front_left.move_h2(x)
        front_right.move_h2(x)
        time.sleep(1.5 * 0.005)

    ############################

    stream = io.BytesIO()
    time.sleep(5)
    ############################################# Open connection to camera
    with picamera.PiCamera() as camera:
        camera.resolution = (RESOLUTION_X, RESOLUTION_Y)
        camera.framerate = 10
        while True:
            camera.capture(stream, format='jpeg', use_video_port=True)
            data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(data, 1)

            # Empty and return the in-memory stream to beginning
            stream.seek(0)
            stream.truncate(0)

            ###################### Image preprocessing   ##########################
            grey_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            display_image = cv2.copyMakeBorder(image, 0, 0, 0, 0, cv2.BORDER_REPLICATE)

            retval, thresh_image = cv2.threshold(grey_image, THRESH_VALUE1, 255, cv2.THRESH_BINARY)
            kernel = np.ones((3, 3), np.uint8)
            thresh_image = cv2.erode(thresh_image, kernel, 7)
            thresh_image = cv2.dilate(thresh_image, kernel, 7)
            retval, thresh_image = cv2.threshold(thresh_image, THRESH_VALUE2, 50, cv2.THRESH_BINARY)

            ########################################################################

            move_forward.move()

            ################## finding image contours
            img, contours, hierarchy = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            ###########################################


            ################## LINE FOLLOWING ###############################
            center_point = (SCAN_POS_X, SCAN_HEIGHT)
            scan_data = scanLine(thresh_image, center_point, SCAN_RADIUS)
            point_on_line = findLine(scan_data, SCAN_POS_X, SCAN_HEIGHT, SCAN_RADIUS)
            returnVal, scan_data = scanCircle(thresh_image, display_image, point_on_line, SCAN_RADIUS_REG, -90)
            previous_point = point_on_line
            last_point = findInCircle(scan_data)
            # cv2.line(display_image, (previous_point[0], previous_point[1]), (last_point[0], last_point[1]),
            #          (255, 255, 255), 1)

            actual_number_of_circles = 0
            for scan_count in range(0, NUMBER_OF_CIRCLES):
                returnVal, scan_data = scanCircle(thresh_image, display_image, last_point, SCAN_RADIUS_REG,
                                                  lineAngle(previous_point, last_point))

                # Only work out the next itteration if our point is within the bounds of the image
                if returnVal == True:
                    actual_number_of_circles += 1
                    previous_point = last_point
                    last_point = findInCircle(scan_data)
                    # cv2.line(display_image, (previous_point[0], previous_point[1]), (last_point[0], last_point[1]),
                    #          (255, 255, 255), 1)
                else:
                    break;

            # # Draw a line from the centre point to the end point where we last found the line we are following
            # cv2.line(display_image, (center_point[0], center_point[1]), (last_point[0], last_point[1]), (0, 0, 255), 1)

            line_scan_length = SCAN_RADIUS_REG * (actual_number_of_circles + 1)
            line_length_from_center = lineLength(center_point, last_point)
            center_y_distance = center_point[1] - last_point[1]
            center_x_distance = center_point[0] - last_point[0]

            angle = lineAngle(center_point, last_point) * -1 - 90,

            if -7 <= angle <= 7:
                if not move_forward.isMoving:
                    move_forward.move()
            elif angle > 7:
                #stop and turnRight
                if move_forward.isMoving:
                    move_forward.stop()
                RotateLeft(front_left, front_right, rear_left, rear_right)
            elif angle < -7:
                #stop and turnLeft
                if move_forward.isMoving:
                    move_forward.stop()
                RotateRight(front_left, front_right, rear_left, rear_right)



            ##################### FOR JUNCTION DETECTION #############################
            if len(contours) > 2:
                if move_forward.isMoving:
                    move_forward.stop()
                SlowStepForward(front_left, front_right, rear_left, rear_right, Factor=0.4)
                SlowStepForward(front_left, front_right, rear_left, rear_right, Factor=0.4)
            
                # Sorting 3 contour values
                contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]
                # Getting least contourArea
                c = contours[-1]
                # Getting extreme points
                extLeft = tuple(c[c[:, :, 0].argmin()][0])
                extRight = tuple(c[c[:, :, 0].argmax()][0])
                extTop = tuple(c[c[:, :, 1].argmin()][0])
            
                # cv2.circle(image, extLeft, 8, (0, 0, 255), -1)
                # cv2.circle(image, extRight, 8, (0, 255, 0), -1)
                # cv2.circle(image, extTop, 8, (255, 0, 0), -1)
            
                l1 = lineLength(extLeft, extTop)
                l2 = lineLength(extRight, extTop)
                l3 = lineLength(extLeft, extRight)
            
                if l1 > l3 and l2 > l3:
                    print "forward"
                    move_forward.move()
                if l1 > l2 and l3 > l2:
                    print "left"
                    RotateLeft(front_left, front_right, rear_left, rear_right)
                if l2 > l1 and l3 > l1:
                    print "right"
                    RotateRight(front_left, front_right, rear_left, rear_right)
            ########################### JUNCTION DETECTION OVER   #################################################


            ########################### LINE FOLLOWING #####################################################

            # Wait for ESC to end program
            c = cv2.waitKey(7) % 0x100
            if c == 27:
                break
        ############################## WHILE LOOP ######################################################
    return;

if __name__ == "__main__":
    main()
