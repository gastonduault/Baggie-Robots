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


# AK.setPitchRangeMoving((0, 7, 12), -50, -90, 0, 1500)
AK = ArmIK()

x_dis = 1500
y_dis = 2500

# [500, 2500]

if __name__ == '__main__':
    # avoid extreme movements and damage to the robot
    x_dis = 500 if x_dis < 500 else x_dis
    x_dis = 2500 if x_dis > 2500 else x_dis

    y_dis = 500 if y_dis < 500 else y_dis
    y_dis = 2500 if y_dis > 2500 else y_dis

    while True:
        Board.setPWMServosPulse([20, 2, 3,int(y_dis), 6,int(x_dis)])
        time.sleep(0.5)



