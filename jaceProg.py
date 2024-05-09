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



def set_gait(phase):
    set_leg(0 + phase if phase < 5 else 0-4 + phase, 0.5, 0.5, 0)
    set_leg(1 + phase if phase < 5 else 1-4 + phase, 0.3, 0, 0)
    set_leg(2 + phase if phase < 5 else 2-4 + phase, 0.5, -0.5, 0)
    set_leg(3 + phase if phase < 5 else 3-4 + phase, 0.2, 0, 0)
    return True

def set_leg(leg_num, length, angle, tilt):
    set_joint((leg_num * 3) + 0, (tilt * 90) + 90)
    set_joint((leg_num * 3) + 1, (length * 90) + 45)
    set_joint((leg_num * 3) + 2, (angle * 90) + 45)
    return True


def set_joint(servo, angle):
    angle_use = (angle * 11111) + 500000
    os.system("echo {} > /sys/class/pwm/pwmchip0/pwm{}/duty_cycle".format(angle_use, servo))
    return True


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

    

    while True:

        for i in range(180):
            set_joint(10, i)
            time.sleep(0.01)


main()
