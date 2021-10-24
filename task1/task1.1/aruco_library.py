#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time


def detect_ArUco(img):
    # function to detect ArUco markers in the image using ArUco library
    # argument: img is the test image
    # return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    # for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    # {0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##
    
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    (corners , ids  , rejectedCorners) = aruco.detectMarkers(
        gray, aruco_dict,parameters=parameters)
    
    Detected_ArUco_markers = { 'markerid' : ids , 'markercorners': corners}
    

    
    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    # function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
    # argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    # return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
    # for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the
    # function should return: {1: 120 , 2: 164}

    ArUco_marker_angles = {}
    ## enter your code here ##
    corners = Detected_ArUco_markers.get('markercorners')
    ids = Detected_ArUco_markers.get('markerid')
    ArUco_marker_angles_list =[]
    
    

    if len(corners) > 0:
        # flatten the ArUco ids list
        ids = ids.flatten()
    # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            mX = int((topLeft[0] + topRight[0])/2.0)
            mY = int((topLeft[1] + topRight[1])/2.0)
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            ArUco_marker_angle = 0
            if(mX != cX):
                radians = math.atan(abs(cY-mY)/abs(mX-cX))
                ArUco_marker_angle = int(math.degrees(radians))
            
            
            
            ArUco_marker_angles_list.append(ArUco_marker_angle)
        ArUco_marker_angles = {'markerid': markerID , 'angle' : ArUco_marker_angles_list}
            
            
        

    # returning the angles of the ArUco markers in degrees as a dictionary
    return ArUco_marker_angles


def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):
    # function to mark ArUco in the test image as per the instructions given in problem statement
    # arguments: img is the test image
    # Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    # ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    # return: image namely img after marking the aruco as per the instruction given in problem statemen
    corners = Detected_ArUco_markers.get('markercorners')
    ids = Detected_ArUco_markers.get('markerid')
    angles = ArUco_marker_angles.get('angle')
    
    
    
    

    if len(corners) > 0:
        ids = ids.flatten()

        for(markerCorner, markerID,markerAngle) in zip(corners, ids, angles):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cv2.circle(img, topLeft, 5, (125, 125, 125), -1)
            cv2.circle(img, topRight, 5, (0, 255, 0), -1)
            cv2.circle(img, bottomRight, 5, (180, 105, 255), -1)
            cv2.circle(img, bottomLeft, 5, (255, 255, 255), -1)
            mX = int((topLeft[0] + topRight[0])/2.0)
            mY = int((topLeft[1] + topRight[1])/2.0)
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.line(img, (cX, cY), (mX, mY), (255, 0, 0), 5)
            cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
            # draw the ArUco marker ID on the img
            cv2.putText(img, str(markerID), ((cX + 10), (cY+10)),
                             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            if((topRight[0] > topLeft[0]) & (topLeft[1] < topRight[1])):
                cv2.putText(img, str(markerAngle), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            elif((topRight[0] > topLeft[0]) & (topLeft[1] == topRight[1])):
                cv2.putText(img, str(90), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            elif((topRight[0] == topLeft[0]) & (topLeft[1] > topRight[1])):
                cv2.putText(img, str(180), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            elif((topRight[0] > topLeft[0]) & (topLeft[1] > topRight[1])):
                cv2.putText(img, str(180 -markerAngle), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            elif((topRight[0] < topLeft[0]) & (topLeft[1] == topRight[1])):
                cv2.putText(img, str(270), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            elif((topRight[0] < topLeft[0]) & (topLeft[1] > topRight[1])):
                cv2.putText(img, str(180 + markerAngle), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)

            elif((topRight[0] < topLeft[0]) & (topLeft[1] < topRight[1])):
                cv2.putText(img, str(360 -markerAngle), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            elif((topRight[0] == topLeft[0]) & (topLeft[1] < topRight[1])):
                cv2.putText(img, str(0), ((cX - 90), (cY-50)), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)
            
        cv2.imshow("img", img)
        
        

    return img