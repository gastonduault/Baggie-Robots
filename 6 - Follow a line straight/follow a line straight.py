#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.Board as Board

from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Misc as Misc
from HiwonderSDK.PID import PID
import HiwonderSDK.mecanum as mecanum


chassis = mecanum.MecanumChassis()

AK = ArmIK()
pitch_pid = PID(P=0.28, I=0.16, D=0.18)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)


def initMove():
    Board.setPWMServoPulse(1, 1500, 800)
    AK.setPitchRangeMoving((0, 7, 11), -60, -90, 0, 1500)
    MotorStop()

line_centerx = -1


def reset():
    global line_centerx
    global __target_color

    line_centerx = -1
    __target_color = ()


def init():
    print("VisualPatrol Init")
    load_config()
    initMove()

__isRunning = False


def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")


def stop():
    global __isRunning
    __isRunning = False
    MotorStop()
    print("VisualPatrol Stop")


def exit():
    global __isRunning
    __isRunning = False
    MotorStop()
    print("VisualPatrol Exit")

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

def MotorStop():
    chassis.set_velocity(0,0,0)
    Board.setMotor(1, 0)
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:
                area_max_contour = c

    return area_max_contour, contour_area_max

img_centerx = 320

def move():
    global line_centerx

    while True:
        if __isRunning:
            if line_centerx != -1:
                if abs(line_centerx - img_centerx) < 20:  # If the line is approximately in the center
                    chassis.set_velocity(50,90,0)
                    # Board.setMotor(1, 50)
                    # Board.setMotor(2, 50)
                    # Board.setMotor(3, 50)
                    # Board.setMotor(4, 50)
                else:
                    MotorStop()
            else:
                MotorStop()
            time.sleep(0.01)
        else:
            MotorStop()
            time.sleep(0.01)


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# Defines regions in the image to focus on for detecting the line.
# Calculates the heights of the regions for processing.


roi = [
    (240, 280,  0, 640, 0.1),
    (340, 380,  0, 640, 0.3),
    (430, 460,  0, 640, 0.6)
]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)

def run(img):
    global line_centerx
    global __target_color

    # Creates a copy of the input image to work on
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning or __target_color == ():
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        # Extracts the region of interest from the blurred frame
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        # Converts the extracted ROI to the LAB color space for better color segmentation
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)
        area_max = 0
        areaMaxContour = 0
        for i in lab_data:
            if i in __target_color:
                detect_color = i
                # frame_mask: Creates a binary mask for the target color
                # eroded: Applies erosion to remove small noise points
                # dilated: Applies dilation to restore the eroded regions and improve the contour's continuity
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))
                # Finds all contours in the binary mask
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                # Retrieves the largest contour using the helper function getAreaMaxContour
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))


        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
        cnt_large, area = getAreaMaxContour(cnts)
        if cnt_large is not None:
            # Calculates the minimum area rectangle that encloses the largest contour
            rect = cv2.minAreaRect(cnt_large)
            # Gets the four vertices of the rectangle
            box = np.int0(cv2.boxPoints(rect))
            # Adjusts the box coordinates to account for the ROI's position
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1) * roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))



            # Draws the contour and the centroid on the image
            cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)

            # Calculates the centroid of the rectangle and adds it to the list of centroids.
            # Accumulates the weighted sum of the centroids' x-coordinates.
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            center_.append([center_x, center_y])
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]
    if weight_sum != 0:
    # calculates the weighted average of the centroids x-coordinates to find the line's center
        line_centerx = int(centroid_x_sum / weight_sum)
    else:
        line_centerx = -1
    return img


def Stop(signum, frame):
    global __isRunning

    __isRunning = False
    MotorStop()

if __name__ == '__main__':
    init()
    start()
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    __target_color = ('blue',)
    while __isRunning:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
