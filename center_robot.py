import sys
sys.path.append('../StanfordQuadruped')

import cv2 as cv
from cv2 import aruco

import numpy as np

import time
import sys
#!/usr/bin/env python3
from UDPComms import Publisher
import time 

# drive_pub = Publisher(8830) = controls movement of pupper (basically mode 1)
# arm_pub = Publisher(8410) = controls more movements of upper (mode 2)
# mode 2 is what you can do when pupper is not in trot mode when using a controller.
drive_pub = Publisher(8830) 
# arm_pub = Publisher(8410)
# L1 = activate/disactivate
# R1 = transition between Rest mode and Trot mode.
# circle = dance or hold for 3 seconds to turn off system
# trinagle  = NOTHING 
# X = jump
# L2 = nothing
# R2 = Nothing
# The range for the following are form (-1, 1)
# ly = forward or backwards
# lx = left or right
# rx = turn left or right (pitch)
# ry = pitches the robot forward

def activate():
    drive_pub.send({"L1": 1, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})
    
def default():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})
  
def trot():
    drive_pub.send({"L1": 0, 
            "R1": 1, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})  
    
def forward():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0.7, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})
def backward():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": -0.7, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0}) 
def right():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 1, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0}) 
def left():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": -1, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0}) 
def turn_left():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": -0.2, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})  
def turn_right():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0.2, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0}) 
def circle_right():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 1, 
            "lx": 0, 
            "rx": 0.2, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0}) 

def look_up():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": 0.2, 
            "dpady": 0, 
            "dpadx": 0})

def look_down():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0, 
            "message_rate": 20, 
            "ry": -0.2, 
            "dpady": 0, 
            "dpadx": 0})

def look_right():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": 0.2, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})
def look_left():
    drive_pub.send({"L1": 0, 
            "R1": 0, 
            "x": 0, 
            "circle": 0, 
            "triangle": 0, 
            "L2": 0, 
            "R2": 0, 
            "ly": 0, 
            "lx": 0, 
            "rx": -0.2, 
            "message_rate": 20, 
            "ry": 0, 
            "dpady": 0, 
            "dpadx": 0})
if __name__ == '__main__':
    centered_x = False
    centered_y = False
    true_x = False
    true_y = False
    activate()
    time.sleep(0.2)
    trot()
    time.sleep(0.2)
    print("starting")
    
    
        
        
         

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
                #print(ids, "  ", corners)
                avg_center = [(top_right[0] + top_left[0])/2, (top_left[1] + bottom_left[1])/2]
                print(avg_center)
                if avg_center[0] < 285:
                    turn_left()
                    centered_x = False
                elif avg_center[0] > 345:
                    turn_right()
                    centered_x = False
                else:
                    centered_x = True
                
                if avg_center[1] < 245:
                    backward()
                    centered_y = False
                elif avg_center[1] > 315:
                    forward()
                    centered_y = False
                else:
                    centered_y = True
                
                if (centered_y and centered_x):
                    print("first center")
                    if avg_center[0] < 305:
                        look_left()
                        true_x = False
                    elif avg_center[0] > 325:
                        look_right()
                        true_x = False
                    else:
                        true_x = True
                    
                    if avg_center[1] < 225:
                        look_down()
                        true_y = False
                    elif avg_center[1] > 245:
                        look_up()
                        true_y = False
                    else:
                        true_y = True
                        
                    if (true_x and true_y):
                        print("true centered")
                        break
        else:
            turn_left()
            

        if (true_x and true_y):
                    print("centered2")
                    break

    print("centered3")                
    default()
    time.sleep(0.2)
    trot()
    time.sleep(0.2)
    activate()
    time.sleep(0.2)
    default()
    print("done")