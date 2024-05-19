GenerateMarkers just needs to be used once to create the AR tags
MarkerDetection is the file where the only actual AR Tag detection is done

IMPORTANT
opencv is too large for it to let me upload to github, so that needs to be installed to the pupper for any of the code to work.
These are the installation steps for windows:
https://docs.opencv.org/4.x/d5/de5/tutorial_py_setup_in_windows.html
https://github.com/opencv/opencv/releases
and for ubuntu: 
https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html
tldr:
installation for ubuntu:
$ sudo apt-get install python3-opencv

for every file, need:
import cv2 as cv