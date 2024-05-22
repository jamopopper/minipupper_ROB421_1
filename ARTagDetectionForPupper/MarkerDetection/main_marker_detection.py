import cv2 as cv
from cv2 import aruco
import numpy as np

###this code detects Aruco tags from a webcam/camera
#using code from YouTube tutorial https://www.youtube.com/watch?v=_6x7pDOJkEk
##https://www.youtube.com/watch?v=P9QZhcteRlU

#getting the aruco dictionary for 4x4 (first 50 AR tags) 
marker_dict =  cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

param_markers = aruco.DetectorParameters()
detector = aruco.ArucoDetector(marker_dict, param_markers)

#capture video
cap = cv.VideoCapture(1) #number indicates what camera you are using. likely 0 by default, 1 or 2 if multiple plugged in

#create a while loop to capture footage. Ends if "q" is pressed on the keyboard
while True:
    #videoCapture just takes one frame at a time, need to store each frame with a while loop
    #ret is validation of frame, 'frame' is pixel data
    ret, frame = cap.read()
    if not ret: 
        break  #if a frame is invalid / corrupt, break
    gray_frame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY) #convert to grayscale
    #define corner positions and identifiers for AR tags
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame) 
    #print corner locations and AR Tag IDs if there is an AR tag in the camera's feed
    if marker_corners:
        for ids, corners in zip(marker_IDs,marker_corners):
            print(ids,"\n",corners,"\n \n")
            #draw a line around the AR tags on the visual
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0,255,255), 4, cv.LINE_4
            )
            corners = corners.reshape(4,2)
            corners = corners.astype(int)
            top_left = corners[0].ravel()
            cv.putText(
                frame,
                "o",
                top_left,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0,255,0),
                2,
                cv.LINE_4,
            )

            top_right = corners[1].ravel()
            cv.putText(
                frame,
                "o",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (10,10,200),
                2,
                cv.LINE_4,
            )

            
    cv.imshow("frame", frame)
    key = cv.waitKey(1)  #if the letter "key" is pressed on the keyboard, stop recording
    if key == ord("q"):  #ord() turns q into unicode
        break #can also ctrl + c in terminal
    
#end recording
cap.release()
cv.destroyAllWindows()

