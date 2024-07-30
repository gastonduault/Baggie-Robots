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

if __name__ == '__main__':
    while True:
        AK.setPitchRangeMoving((0, 7, 12), -50, -90, 0, 1500)
        time.sleep(0.5)



