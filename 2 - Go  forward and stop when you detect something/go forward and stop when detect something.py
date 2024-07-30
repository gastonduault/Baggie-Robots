#!/usr/bin/python3
#coding=utf8
import sys
sys.path.append('/home/pi/MasterPi')
import cv2
import time
import signal
import Camera
import numpy as np
import pandas as pd
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.mecanum as mecanum

# 超声波避障

chassis = mecanum.MecanumChassis()
AK = ArmIK()

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

HWSONAR = None
Threshold = 15.0
TextColor = (0, 255, 255)
TextSize = 12

speed = 40
__isRunning = False
__until = 0


# 夹持器夹取时闭合的角度
servo1 = 1500

# 初始位置
def initMove():
    chassis.set_velocity(0,0,0)
    Board.setPWMServoPulse(1, servo1, 300)
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500)

# 变量重置
def reset():
    global __isRunning
    global Threshold
    global speed
    global stopMotor
    global wait
    global forward
    global old_speed

    speed = 100
    old_speed = 0
    Threshold = 40.0
    wait = True
    forward = True
    stopMotor = True
    __isRunning = False

# app初始化调用
def init():
    initMove()
    reset()

__isRunning = False
def start():
    global __isRunning
    global stopMotor
    global forward
    global wait

    wait = True
    forward = True
    stopMotor = True
    __isRunning = True
    print("Avoidance Start")

def stop():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,0,0)
    print("Avoidance Stop")

def exit():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,0,0)
    HWSONAR.setPixelColor(0, Board.PixelColor(0, 0, 0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0, 0, 0))
    print("Avoidance Exit")

def setSpeed(args):
    global speed
    speed = int(args[0])
    return (True, ())

def setThreshold(args):
    global Threshold
    Threshold = args[0]
    return (True, (Threshold,))

def getThreshold(args):
    global Threshold
    return (True, (Threshold,))


wait = True
forward = True
old_speed = 0
stopMotor = True
distance_data = []

def run(img):
    global wait
    global speed
    global __until
    global __isRunning
    global HWSONAR
    global Threshold
    global distance_data
    global stopMotor
    global forward
    global old_speed

    dist = HWSONAR.getDistance() / 10.0

    distance_data.append(dist)
    data = pd.DataFrame(distance_data)
    data_ = data.copy()
    u = data_.mean()
    std = data_.std()

    data_c = data[np.abs(data - u) <= std]
    distance = data_c.mean()[0]

    if len(distance_data) == 5:
        distance_data.remove(distance_data[0])

    if __isRunning:
        # if the current distance is more small than the limit
        if distance <= Threshold:
            if wait:
                wait = False
                forward = True
                stopMotor = True
                chassis.set_velocity((5* speed/6),90,0)
                time.sleep(0.2)
                chassis.set_velocity((4* speed/6),90,0)
                time.sleep(0.2)
                chassis.set_velocity((3* speed/6),90,0)
                time.sleep(0.2)
                chassis.set_velocity((2* speed/6),90,0)
                time.sleep(0.2)
                chassis.set_velocity(speed/6,90,0)
                time.sleep(0.2)
                chassis.set_velocity(0,0,0)
                time.sleep(0.2)

        else:
            if forward:
                wait = True
                forward = False
                stopMotor = True
                chassis.set_velocity(speed,90,0)
    else:
        if stopMotor:
            stopMotor = False
            chassis.set_velocity(0,0,0)  # 关闭所有电机
        wait = True
        forward = True
        time.sleep(0.03)

    return cv2.putText(img, "Dist:%.1fcm"%distance, (30, 480-30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TextColor, 2)  # 把超声波测距值打印在画面上


#关闭前处理
def Stop(signum, frame):
    global __isRunning

    __isRunning = False
    print('关闭中...')
    chassis.set_velocity(0,0,0)  # 关闭所有电机

if __name__ == '__main__':
    init()
    start()
    HWSONAR = Sonar.Sonar()
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    while __isRunning:
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
    
    