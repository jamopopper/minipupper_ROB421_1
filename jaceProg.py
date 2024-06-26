import sys
import jaceCommands as jc

sys.path.append('../StanfordQuadruped')

import numpy as np
import time
import os
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


    store = jc.stand()
    state.joint_angles = store
    jc.set_servos(hardware_interface, state.joint_angles)
    time.sleep(1)
    

    while True:

        ## Stand
        storeDefault = jc.stand()
        storeA = jc.stand(5)
        storeB = jc.stand(245)
        jc.keyframe(1, storeDefault, storeA, hardware_interface)
        jc.keyframe(1, storeA, storeB, hardware_interface)
        jc.keyframe(1, storeB, storeDefault, hardware_interface)
        time.sleep(1)

        ## Walk forward, right, backward, left
        print("forward")
        # jc.walk_control(0, 0.75, 4, hardware_interface)      
        # time.sleep(1)
        print("right")
        jc.walk_control(0.25, 0.75, 4, hardware_interface)      
        time.sleep(1)
        print("backward")
        # jc.walk_control(0.5, 0.75, 4, hardware_interface)      
        # time.sleep(1)
        print("left")
        jc.walk_control(0.75, 0.75, 4, hardware_interface)      
        time.sleep(1)

        ## Look around
        store1 = jc.look_around(127, 1)
        store2 = jc.look_around(127, -1)
        store3 = jc.look_around(127, 0)
        jc.keyframe(0.5, store3, store1, hardware_interface)
        jc.keyframe(1, store1, store2, hardware_interface)
        jc.keyframe(0.5, store2, store3, hardware_interface)
        time.sleep(1)
        
        ## Turn a little
        jc.turn_around(127, 1, hardware_interface)
        time.sleep(0.5)
        jc.turn_around(127, -1, hardware_interface)
        jc.turn_around(127, -1, hardware_interface)
        time.sleep(0.5)
        jc.turn_around(127, 1, hardware_interface)
        time.sleep(1)




main()
