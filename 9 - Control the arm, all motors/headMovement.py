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

chassis = mecanum.MecanumChassis()
AK = ArmIK()

if __name__ == '__main__':
    while True:
        Board.setPWMServoPulse(3, 1200, 200)
        time.sleep(0.5)
        Board.setPWMServoPulse(6, 1300, 400)
        time.sleep(0.5)
        Board.setPWMServoPulse(3, 200, 200)
        time.sleep(0.5)
        Board.setPWMServoPulse(6, 1700, 400)
        time.sleep(0.5)
    chassis.set_velocity(0,0,0)





