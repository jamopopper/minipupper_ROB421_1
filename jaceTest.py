import numpy as np
import time
from StanfordQuadruped.src.IMU import IMU
from StanfordQuadruped.src.Controller import Controller
from StanfordQuadruped.src.JoystickInterface import JoystickInterface
from StanfordQuadruped.src.State import BehaviorState, State
from StanfordQuadruped.MangDang.mini_pupper.HardwareInterface import HardwareInterface
from StanfordQuadruped.MangDang.mini_pupper.Config import Configuration
from StanfordQuadruped.pupper.Kinematics import four_legs_inverse_kinematics
from StanfordQuadruped.MangDang.mini_pupper.display import Display
from StanfordQuadruped.src.MovementScheme import MovementScheme
from StanfordQuadruped.src.danceSample import MovementLib

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

    


main()