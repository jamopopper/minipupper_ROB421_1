import cv2 as cv
from cv2 import aruco

###this code generates Aruco tags to be used by a webcam/camera 
###this is continued in main_markers_detection.py (MarkerDetection folder)

#using code from YouTube tutorial https://www.youtube.com/watch?v=_6x7pDOJkEk

#getting the aruco dictionary for 4x4 (first 50 AR tags) 
marker_dict =  cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
#tutorial used old version code: "marker_dict = aruco.Dictionary(aruco.DICT_4X4_50)"

#specify size
MARKER_SIZE = 400 #pixels
#generate unique ID
for id in range(20):
    #create function that draws marker on image
    marker_image = aruco.generateImageMarker(marker_dict, id, MARKER_SIZE)
    #tutorial used old version code: "marker_image = aruco.drawDetectedMarkers(marker_dict, id, MARKER_SIZE)"

    #show image
    cv.imshow("img",marker_image)
    #save image to a folder as marker_[id].png
    cv.imwrite(f"markers/marker_{id}.png", marker_image) 
    #cv.waitKey(0)
    #break    #these two lines just allow you to see the first marker generated

###continue to MarkerDetection folder
