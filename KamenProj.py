import sys
import NicProject as nic
sys.path.append('../StanfordQuadruped')

import cv2 as cv
from cv2 import aruco

import numpy as np

import time
import os
import sys
"""
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import BehaviorState, State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics, leg_explicit_inverse_kinematics
from MangDang.mini_pupper.display import Display
from src.MovementScheme import MovementScheme
from src.danceSample import MovementLib
from UDPComms import Publisher
"""

def walk0(array, phase):
    servo_sin = np.sin((6.28) * phase)
    servo_cos = (-np.cos((6.28) * phase) + 1) / 2
    store = jc.stand(array, servo_cos*255, 0, 0, 0)
    return store

def walk2(array, phase):
    servo_sin = np.sin((6.28) * phase)
    servo_cos = (-np.cos((6.28) * phase) + 1) / 2
    store = jc.stand(array, servo_cos*255, 0, 0, 2)
    return store

def walk1(array, phase):
    servo_sin = np.sin((6.28) * phase)
    servo_cos = (-np.cos((6.28) * phase) + 1) / 2
    store = jc.stand(array, servo_cos*255, 0, 0, 1)
    return store

def walk3(array, phase):
    servo_sin = np.sin((6.28) * phase)
    servo_cos = (-np.cos((6.28) * phase) + 1) / 2
    store = jc.stand(array, servo_cos*255, 0, 0, 3)
    return store

def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    #Create movement group scheme instance and set a default false state
    movementCtl = MovementScheme(MovementLib)
    dance_active_state = False

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)


    store = stand(state.joint_angles)
    state.joint_angles = store
    hardware_interface.set_actuator_postions(state.joint_angles)
    time.sleep(1)
    
    """
    while True:
        for i in range(128):
            store = walk0(state.joint_angles, i/128)
            state.joint_angles = store
            hardware_interface.set_actuator_postions(state.joint_angles)
            time.sleep(0.01)
            store = walk2(state.joint_angles, i/128)
            state.joint_angles = store
            hardware_interface.set_actuator_postions(state.joint_angles)
            """
    

###CAMERA CODE (from pupper_detection.py)

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(0, cv.CAP_V4L2) #added  (... , cv.CAP_V4L2)   was giving an error in GStramer pipeline

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            cv.putText(
                frame,
                f"id: {ids[0]}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )
            print(ids, "  ", corners)
            if ids == 18:
                nic.main()
                
