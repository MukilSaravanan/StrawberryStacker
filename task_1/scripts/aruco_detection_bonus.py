#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from aruco_library import *


 

video=cv2.VideoCapture("test_video.mp4")
fps=int(video.get(cv2.CAP_PROP_FPS))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_width = int(video.get(3))
frame_height = int(video.get(4))
size = (frame_width, frame_height)
out = cv2.VideoWriter('outpu.avi', fourcc, 10.0, size)
while(True):
    res,frame=video.read()
    time.sleep(1/fps)

    detected_aruco_markers=detect_ArUco(frame)
    if detected_aruco_markers==1:
        continue
    detected_angles=Calculate_orientation_in_degree(detected_aruco_markers)


    res=mark_ArUco(frame,detected_aruco_markers,detected_angles)
    cv2.imshow("Dotted Image",res)
    out.write(res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
video.release()
cv2.destroyAllWindows()
