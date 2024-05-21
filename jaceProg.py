import sys

sys.path.append('../StanfordQuadruped')

import numpy as np
import time
import os
import jaceCommands as jc
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


    store = jc.stand(state.joint_angles)
    state.joint_angles = store
    jc.set_servos(hardware_interface, state.joint_angles)
    time.sleep(1)
    

    while True:
        for i in range(128):
            store = jc.dance(state.joint_angles, i/128)
            state.joint_angles = store
            jc.set_servos(hardware_interface, state.joint_angles)

        # for i in range(128):
        #     # store = dance(state.joint_angles, i/128)
        #     store = walk_control(state.joint_angles, 0, 0, i/128)

        #     state.joint_angles = store

        #     set_servos(hardware_interface, state.joint_angles)

        # for i in reversed(range(128)):
        #     # store = dance(state.joint_angles, i/128)
        #     store = walk_control(state.joint_angles, 0, 0, i/128)

        #     state.joint_angles = store

        #     set_servos(hardware_interface, state.joint_angles)

main()
