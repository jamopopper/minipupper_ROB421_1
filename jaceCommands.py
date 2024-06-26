import sys

sys.path.append('../StanfordQuadruped')

import numpy as np
import time


def set_servos(hw_face, state): # WORKS
    hw_face.set_actuator_postions(state)
    time.sleep(0.02)
    return True

def keyframe(duration, start_pos, end_pos, hw_face): # WORKS
    start_time = time.time()
    array = np.zeros((3,4))
    while (start_time + duration) > time.time():
    #while (time.time() - start_time) < duration:
        current_step = (time.time() - start_time) / duration
        for i in range(3):
            for j in range(4):
                array[i, j] = (start_pos[i, j] * (1-current_step)) + (end_pos[i, j] * current_step)
        set_servos(hw_face, array)
    set_servos(hw_face, end_pos)


    return array

def look_around(height=127, offset=0): # WORKS
    # positive looks right, negative looks left
    # 0-1
    store1 = stand(height, 0, offset * 63, 7)
    store2 = stand(height, 0, offset * -63, 8)
    return (store1 + store2)

def turn_around(height, offset, hw_face): # WORKS

    store1a = stand(height, 0, offset * 63, 0) # front right
    store1b = stand(height, 0, offset * 63, 1) # front left

    store2a = stand(height, 0, offset * -63, 2) # back right
    store2b = stand(height, 0, offset * -63, 3) # back left

    leg_up_1 = stand(63, leg=5) # front left and back right
    leg_up_2 = stand(63, leg=6) # front right and back left

    leg_set_1 = stand(127, leg=5) # front left and back right
    leg_set_2 = stand(127, leg=6) # front right and back left

    set_servos(hw_face, (store1a + store1b + store2a + store2b))
    keyframe(0.1, (store1a + store1b + store2a + store2b), (leg_up_2 + store1a + store2b), hw_face)
    keyframe(0.1, (leg_up_2 + store1a + store2b), (leg_set_2 + store1a + store2b), hw_face)
    keyframe(0.1, (leg_set_2 + store1a + store2b), (leg_set_2 + leg_up_1), hw_face)
    keyframe(0.1, (leg_set_2 + leg_up_1), (leg_set_2 + leg_set_1), hw_face)



def stand(height=127, lean=0, roll=0, leg=4, offset=0): # WORKS
    # array is the given servo array
    # height (default=127) goes from 0-255
    # lean (default=0) goes from 0-63 positive and negative, positive moves body forward
    # roll (default=0) goes from 0-63 positive and negative, positive moves body to the right
    # leg (default=4) specifies setting values to a specific leg

    # shoulders (0) go from 0.5 to -0.5
    # upper joints (1) go from 0.1 to 0.728
    # lower joints (2) go from -0.1 to -0.728
    # leg 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left
    # leg 4 is all legs, 5 is front-right and back-left, 6 is front-left and back-right
    # leg 7 is front legs, leg 8 is back legs

    # TEMP
    offset = 0
    # TEMP

    servo_offset = 0.2
    lean_scale = 0.5
    roll_shoulder_scale = 0.4
    roll_leg_scale = 0.15
    array = np.zeros((3,4))

    print(height, lean, roll, leg, offset)

    if leg == 8:
        array += stand(height - offset, lean, roll, 2)
        array += stand(height + offset, lean, roll, 3)
    elif leg == 7:
        array += stand(height - offset, lean, roll, 0)
        array += stand(height + offset, lean, roll, 1)
    elif leg == 6:
        array += stand(height - offset, lean, roll, 1)
        array += stand(height + offset, lean, roll, 2)
    elif leg == 5:
        array += stand(height - offset, lean, roll, 0)
        array += stand(height + offset, lean, roll, 3)
    elif leg == 4:
        array += stand(height, lean, roll, 0)
        array += stand(height, lean, roll, 1)
        array += stand(height, lean, roll, 2)
        array += stand(height, lean, roll, 3)
    else:
        side = -1
        if leg == 0 or leg == 2: side = 1
        array[0, leg] = (roll/64) * roll_shoulder_scale
        array[1, leg] = ((3.14/2.7) * ((256-height)/256) + servo_offset + ((lean/64) * lean_scale) + ((roll/64) * roll_leg_scale * side))
        array[2, leg] = -((3.14/2.7) * ((256-height)/256) + servo_offset + ((roll/64) * roll_leg_scale * side))

    return array

def walk_control(direction, distance, steps, hw_face):
    # direction is the direction you want to step toward and goes from 0-1
    # distance is how far the steps should be

    #DEPREICATED# direction 0 is forward, 0.25 is right, 0.5 is backward, 0.75 is left
    # direction 0 is forward, 1 is right, 2 is left, 3 is backward
    # lead_set when 0 is front-left and back-right, and 1 is front-right and back-left
    # frame when 0 is back step, 0.5 is neutral, and 1 is fully stepped forward

    stationary_step = stand(127, 0, 0, 4)
    print("created stationary")

    #  np.cos(direction * 6.28) + np.sin(direction * 6.28)
    #  np.cos(direction * 6.28) + np.sin(direction * 6.28)
    #  -np.cos(direction * 6.28) + -np.sin(direction * 6.28)
    #  -np.cos(direction * 6.28) + -np.sin(direction * 6.28)

    full_step = stand(127, np.cos(direction * 6.28) * 63 * distance, np.sin(direction * 6.28) * 63 * distance, 5, (np.cos(direction * 6.28) + -np.sin(direction * 6.28)) * 63)
    full_step += stand(127, -np.cos(direction * 6.28) * 63 * distance, -np.sin(direction * 6.28) * 63 * distance, 6, (np.cos(direction * 6.28) + -np.sin(direction * 6.28)) * 63)
    print("created full step")

    full_inv_step = stand(127, np.cos(direction * 6.28) * 63 * distance, np.sin(direction * 6.28) * 63 * distance, 6, (np.cos(direction * 6.28) + -np.sin(direction * 6.28)) * 63)
    full_inv_step += stand(127, -np.cos(direction * 6.28) * 63 * distance, -np.sin(direction * 6.28) * 63 * distance, 5, (np.cos(direction * 6.28) + -np.sin(direction * 6.28)) * 63)
    print("created full inv step")

    mid_step = stand(127, 0, 0, 5)
    mid_step += stand(63, 0, 0, 6)
    print("created mid step")

    mid_inv_step = stand(127, 0, 0, 6)
    mid_inv_step += stand(63, 0, 0, 5)
    print("created mid inv step")

    
    keyframe(0.2, stationary_step, mid_step, hw_face)

    for i in range(steps):
        print(i)
        for j in range(4):
            if (j == 0):
                keyframe(0.05, mid_step, full_step, hw_face)
                print("in step")
                time.sleep(0.05)
            elif (j == 1):
                keyframe(0.1, full_step, mid_inv_step, hw_face)
                print("to out step")
            elif (j == 2):
                keyframe(0.05, mid_inv_step, full_inv_step, hw_face)
                print("out step")
                time.sleep(0.05)
            elif (j == 3):
                keyframe(0.1, full_inv_step, mid_step, hw_face)
                print("to out step")
            else:
                time.sleep(1)
                print("bad times ahead!")

    keyframe(0.2, mid_step, stationary_step, hw_face)

    ### DEPRICATED ###
    # local_pi = 3.14
    # local_pi_double = local_pi * 2

    # array1 = np.zeros((3,4))
    # array2 = np.zeros((3,4))

    # if lead_set == 0:
    #     array1 = stand(127 - (np.sin(local_pi*frame)), np.cos(local_pi * direction) * 2*(frame-0.5) * 64, np.sin(local_pi * direction) * 2*(frame-0.5) * 64, 5)
    #     array2 = stand(127, -np.cos(local_pi * direction) * 2*(frame-0.5) * 64, -np.sin(local_pi * direction) * 2*(frame-0.5) * 64, 6)
    # else:
    #     array1 = stand(127 - (np.sin(local_pi*frame)), np.cos(local_pi * direction) * 2*(frame-0.5) * 64, np.sin(local_pi * direction) * 2*(frame-0.5) * 64, 6)
    #     array2 = stand(127, -np.cos(local_pi * direction) * 2*(frame-0.5) * 64, -np.sin(local_pi * direction) * 2*(frame-0.5) * 64, 5)

    # return (array1 + array2)

def dance(frame): # WORKS
    # frame goes from 0-1

    # array is the given servo array
    # moves in a circle

    servo_sin = np.sin((6.28) * frame)
    servo_cos = (-np.cos((6.28) * frame) + 1) / 2
    array = stand(height=servo_cos*255, roll=servo_sin*63)
    return array






