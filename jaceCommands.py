import sys

sys.path.append('../StanfordQuadruped')

import numpy as np
import time


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


def walk_control(array, direction, lead_set, frame):
    # array is the given servo array
    # direction is the direction you want to step toward and goes from 0-1
    # lead_set is the set of feet that are leading the step
    # frame is how far you are through the step and goes from 0-1

    # direction 0 is forward, 0.25 is right, 0.5 is backward, 0.75 is left
    # lead_set when 0 is front-left and back-right, and 1 is front-right and back-left
    # frame when 0 is back step, 0.5 is neutral, and 1 is fully stepped forward

    store = array

    if lead_set == 0:
        store = stand(array, 127 - (np.sin(3.14*frame)), np.sin(6.28 * direction) * 2*(frame-0.5) * 64, np.cos(6.28 * direction) * 2*(frame-0.5) * 64, 5)
        array = stand(store, 127, -np.sin(6.28 * direction) * 2*(frame-0.5) * 64, -np.cos(6.28 * direction) * 2*(frame-0.5) * 64, 6)
        store = array
    else:
        store = stand(array, 127 - (np.sin(3.14*frame)), np.sin(6.28 * direction) * 2*(frame-0.5) * 64, np.cos(6.28 * direction) * 2*(frame-0.5) * 64, 6)
        array = stand(store, 127, -np.sin(6.28 * direction) * 2*(frame-0.5) * 64, -np.cos(6.28 * direction) * 2*(frame-0.5) * 64, 5)
        store = array

    return array


def keyframe(array, duration, end_pos, hw_face):
    start_time = time.time()
    while (start_time + duration) > time.time():
    #while (time.time() - start_time) < duration:
        current_step = (time.time() - start_time) / duration
        print(current_step)
        store = np.add(np.multiply(array, current_step), np.multiply(end_pos, 1-current_step))
        set_servos(hw_face, store)


    store = array
    return store


def set_servos(hw_face, state):
    hw_face.set_actuator_postions(state)
    time.sleep(0.05)
    return True

def dance(array, frame): 
    # array is the given servo array
    # frame goes from 0-1

    # moves in a circle

    servo_sin = np.sin((6.28) * frame)
    servo_cos = (-np.cos((6.28) * frame) + 1) / 2
    store = stand(array, height=servo_cos*255, roll=servo_sin*63)
    return store

