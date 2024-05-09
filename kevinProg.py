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

config = Configuration()
state = State()
command = Command()
hardware_interface = HardwareInterface()


# # Create controller
controller = Controller(
     config,
     four_legs_inverse_kinematics,
)

state.quat_orientation = np.array([1,0,0,0])

state.behavior_state = BehaviorState.TROT

command.horizontal_velocity = np.array([0.1,0])

controller.run(state, command)
hardware_interface.set_actuator_postions(state.joint_angles)