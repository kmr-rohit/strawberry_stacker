#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR aruco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from aruco_library import detect_ArUco
from aruco_library import Calculate_orientation_in_degree
from aruco_library import mark_ArUco


cap = cv2.VideoCapture(
    "/home/cybernauts/catkin_ws/src/strawberry_stacker/task_1/scripts/test_video.mp4")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
while(cap.isOpened()):
    ret, frame = cap.read()
    
    if ret == True:

        out.write(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame', gray)
        
        Detected_ArUco_markers = detect_ArUco(frame)
       
        angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
        frame = mark_ArUco(frame, Detected_ArUco_markers, angle)
        if cv2.waitKey(25) & 0xFF == ord('q'):

            break  
    else:
        break
cap.release()
out.release()
cv2.destroyAllWindows() 