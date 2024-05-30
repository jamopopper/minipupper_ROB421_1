import sys

sys.path.append('../StanfordQuadruped')

import numpy as np
import time


def set_servos(hw_face, state):
    hw_face.set_actuator_postions(state)
    time.sleep(0.05)
    return True

def keyframe(duration, start_pos, end_pos, hw_face):
    start_time = time.time()
    array = np.zeros((3,4))
    while (start_time + duration) > time.time():
    #while (time.time() - start_time) < duration:
        current_step = (time.time() - start_time) / duration
        print(current_step)
        for i in range(3):
            for j in range(4):
                array[i, j] = (start_pos[i, j] * (1-current_step)) + (end_pos[i, j] * current_step)
        print(array)
        set_servos(hw_face, array)


    return array

def look_right(height=127):
    store = np.zeros((3,4))
    store = stand(height, 0, 63, 7)
    store = stand(height, 0, -63, 8)
    return store

def look_left(height=127):
    store = np.zeros((3,4))
    store = stand(height, 0, -63, 7)
    store = stand(height, 0, -63, 8)
    return store


def stand(height=127, lean=0, roll=0, leg=4): 
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
    # leg 7 is front legs, leg 8 is back legs

    array = np.zeros((3,4))

    if leg == 8:
        array[0, 2] = (roll/64) * 0.4
        array[0, 3] = (roll/64) * 0.4
        array[1, 2] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 3] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[2, 2] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 3] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    elif leg == 7:
        array[0, 0] = (roll/64) * 0.4
        array[0, 1] = (roll/64) * 0.4
        array[1, 0] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 1] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[2, 0] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 1] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    elif leg == 6:
        array[0, 1] = (roll/64) * 0.4
        array[0, 2] = (roll/64) * 0.4
        array[1, 1] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 2] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[2, 1] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 2] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    elif leg == 5:
        array[0, 0] = (roll/64) * 0.4
        array[0, 3] = (roll/64) * 0.4
        array[1, 0] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 3] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[2, 0] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 3] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    elif leg == 4:
        array[0, 0] = (roll/64) * 0.4
        array[0, 1] = (roll/64) * 0.4
        array[0, 2] = (roll/64) * 0.4
        array[0, 3] = (roll/64) * 0.4
        array[1, 0] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 1] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 2] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[1, 3] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[2, 0] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 1] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 2] = -((3.14/2.7) * ((256-height)/256) + 0.2)
        array[2, 3] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    else:
        array[0, leg] = (roll/64) * 0.4
        array[1, leg] = ((3.14/2.7) * ((256-height)/256) + 0.2 + ((lean/64) * 0.5))
        array[2, leg] = -((3.14/2.7) * ((256-height)/256) + 0.2)
    
    return array

def walk_control(direction, lead_set, frame):
    # array is the given servo array
    # direction is the direction you want to step toward and goes from 0-1
    # lead_set is the set of feet that are leading the step
    # frame is how far you are through the step and goes from 0-1

    #DEPRICATED# direction 0 is forward, 0.25 is right, 0.5 is backward, 0.75 is left
    # direction 0 is forward, 1 is right, 2 is left, 3 is backward
    # lead_set when 0 is front-left and back-right, and 1 is front-right and back-left
    # frame when 0 is back step, 0.5 is neutral, and 1 is fully stepped forward

    array = np.zeros((3,4))



    ### PERFECT GAIT BUT IM GIVING UP ON IT

    # if lead_set == 0:
    #     array = stand(127 - (np.sin(3.14*frame)), np.cos(6.28 * direction) * 2*(frame-0.5) * 64, np.sin(6.28 * direction) * 2*(frame-0.5) * 64, 5)
    #     array = stand(127, -np.cos(6.28 * direction) * 2*(frame-0.5) * 64, -np.sin(6.28 * direction) * 2*(frame-0.5) * 64, 6)
    # else:
    #     array = stand(127 - (np.sin(3.14*frame)), np.cos(6.28 * direction) * 2*(frame-0.5) * 64, np.sin(6.28 * direction) * 2*(frame-0.5) * 64, 6)
    #     array = stand(127, -np.cos(6.28 * direction) * 2*(frame-0.5) * 64, -np.sin(6.28 * direction) * 2*(frame-0.5) * 64, 5)

    return array

def dance(frame): 
    # frame goes from 0-1

    # array is the given servo array
    # moves in a circle

    servo_sin = np.sin((6.28) * frame)
    servo_cos = (-np.cos((6.28) * frame) + 1) / 2
    array = stand(height=servo_cos*255, roll=servo_sin*63)
    return array






