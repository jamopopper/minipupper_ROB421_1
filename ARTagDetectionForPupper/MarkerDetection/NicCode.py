import sys

sys.path.append('../StanfordQuadruped')

import numpy as np
import time
import os
import sys
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

def servo_smoothing(next_array, previous_array, smooth_ratio=0.5):
    # next_array is the positions you want to set the servos to
    # previous_array is the positions currently set for the servos
    # smooth_ratio goes from 0-1, larger is less smoothing

    store = (next_array * smooth_ratio) + (previous_array * (1-smooth_ratio))
    print("PREV: ", previous_array, "NOW: ", store, "NEXT: ", next_array)
    print("\n")
    return store

def stand(array, height=127, lean=0, roll=0, leg=4): 
    # array is the given servo array
    # height (default=127) goes from 0-255
    # lean (default=0) goes from 0-63 positive and negative, positive moves body forward
    # roll (default=0) goes from 0-63 positive and negative, positive moves body to the right
    # leg (default=4) specifies setting values to a specific leg

    # shoulders (0) go from 0.5 to -0.5
    # upper joints (1) go from 0.1 to 0.728
    # lower joints (2) go from -0.1 to -0.728
    # leg 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left
    # leg 4 is all legs, 5 is front-left and back-right, 6 is front-right and back-left

    copy = array
    
    if leg == 6:
        copy[0, 1] = (roll/64) * 0.4
        copy[0, 2] = (roll/64) * 0.4
        copy[1, 1] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[1, 2] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[2, 1] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        copy[2, 2] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    elif leg == 5:
        copy[0, 0] = (roll/64) * 0.4
        copy[0, 3] = (roll/64) * 0.4
        copy[1, 0] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[1, 3] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[2, 0] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        copy[2, 3] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    elif leg == 4:
        copy[0, 0] = (roll/64) * 0.4
        copy[0, 1] = (roll/64) * 0.4
        copy[0, 2] = (roll/64) * 0.4
        copy[0, 3] = (roll/64) * 0.4
        copy[1, 0] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[1, 1] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[1, 2] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[1, 3] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[2, 0] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        copy[2, 1] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        copy[2, 2] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        copy[2, 3] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    else:
        copy[0, leg] = (roll/64) * 0.4
        copy[1, leg] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        copy[2, leg] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    
    return copy

def base(array):
    store = stand(array, 127, 0, 0, 4)
    return store

def sidestep5_1(array):
    store = stand(array, 0, 0, 0, 5)
    return store

def sidestep5_2(array):
    store = stand(array, 127, 0, 30, 5)
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
    

    while True:
        for i in range(1):
            store = base(state.joint_angles)
            state.joint_angles = store
            hardware_interface.set_actuator_postions(state.joint_angles)
            time.sleep(0.01)
        for i in range(1):
            store = sidestep5_1(state.joint_angles)
            state.joint_angles = store
            hardware_interface.set_actuator_postions(state.joint_angles)
            time.sleep(0.01)
        for i in range(1):
            store = sidestep5_2(state.joint_angles)
            state.joint_angles = store
            hardware_interface.set_actuator_postions(state.joint_angles)
            time.sleep(0.01)
    
    
main()
