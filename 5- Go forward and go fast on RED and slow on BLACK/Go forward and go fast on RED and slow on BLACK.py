#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import threading
import yaml_handle
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum



AK = ArmIK()
chassis = mecanum.MecanumChassis()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

__target_color = ('red', 'green', 'blue')
def setTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max


servo1 = 1500


def initMove():
    Board.setPWMServoPulse(1, servo1, 300)
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500)


def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True

# 变量重置
def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True

# app初始化调用
def init():
    print("ColorDetect Init")
    load_config()
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Stop")

def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
def move():
    global rect
    global _stop
    global get_roi
    global __isRunning
    global unreachable
    global detect_color
    global start_pick_up
    global rotation_angle
    global world_X, world_Y


    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:

                set_rgb(detect_color)
                setBuzzer(0.1)

                if detect_color == 'red' :
                    chassis.set_velocity(100,90,0)
                    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 500)
                    detect_color = 'None'
                    start_pick_up = False
                    set_rgb(detect_color)

                if detect_color == 'black' :
                    chassis.set_velocity(40,90,0)
                    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 500)
                    detect_color = 'None'
                    start_pick_up = False
                    set_rgb(detect_color)

                else:
                    chassis.set_velocity(0,90,0)
                    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 500)
                    detect_color = 'None'
                    start_pick_up = False
                    set_rgb(detect_color)
            else:
                time.sleep(0.01)
        else:
            if _stop:
                print('ok')
                _stop = False
                initMove()
                time.sleep(1.5)
            time.sleep(0.01)


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]

def run(img):
    global roi
    global rect
    global count
    global get_roi
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global last_x, last_y
    global rotation_angle
    global world_X, world_Y
    global start_count_t1, t1
    global detect_color, draw_color, color_list

    if not __isRunning:
        return img
    else:
        # Creates a copy of the frame to avoid modifying the original image.
        # Resizes the frame to a predefined size (640x480)
        # Applies Gaussian blur to the resized frame to reduce noise and smoothen the image.

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
        if not start_pick_up:
            # Loop Through Colors: Iterates through the colors specified in lab_data that are also in __target_color.
            # Create Mask: Creates a binary mask where pixels within the specified LAB range for the current color are white, and all other pixels are black.
            # Find Contours: Finds contours in the processed binary mask.
            for i in lab_data:
                if i in __target_color:
                    frame_mask = cv2.inRange(frame_lab,
                                             (lab_data[i]['min'][0],
                                              lab_data[i]['min'][1],
                                              lab_data[i]['min'][2]),
                                             (lab_data[i]['max'][0],
                                              lab_data[i]['max'][1],
                                              lab_data[i]['max'][2]))
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                    areaMaxContour, area_max = getAreaMaxContour(contours)
                    if areaMaxContour is not None:
                        if area_max > max_area:
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            # Finds the contour with the largest area using the getAreaMaxContour function.
            if max_area > 2500:
                rect = cv2.minAreaRect(areaMaxContour_max)
                box = np.int0(cv2.boxPoints(rect))

                cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
                if not start_pick_up:
                    if color_area_max == 'red':
                        color = 1
                    elif color_area_max == 'green':
                        color = 2
                    elif color_area_max == 'blue':
                        color = 3
                    else:
                        color = 0
                    color_list.append(color)
                    if len(color_list) == 3:

                        color = int(round(np.mean(np.array(color_list))))
                        color_list = []
                        start_pick_up = True
                        if color == 1:
                            detect_color = 'red'
                            draw_color = range_rgb["red"]
                        elif color == 2:
                            detect_color = 'green'
                            draw_color = range_rgb["green"]
                        elif color == 3:
                            detect_color = 'blue'
                            draw_color = range_rgb["blue"]
                        else:
                            detect_color = 'None'
                            draw_color = range_rgb["black"]
            else:
                if not start_pick_up:
                    draw_color = (0, 0, 0)
                    detect_color = "None"

        cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)

        return img

if __name__ == '__main__':
    init()
    start()
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    while True:
        ret,img = cap.read()
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

