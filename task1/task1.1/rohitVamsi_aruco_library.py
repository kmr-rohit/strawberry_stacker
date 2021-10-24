#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2.cv2 as cv
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##
    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    bboxes, ids, rejected = aruco.detectMarkers(imgGray,arucoDict)

    if len(bboxes) != 0:
        for bbox, box_id in zip(bboxes,ids):
    	    Detected_ArUco_markers[box_id[0]] = bbox
		
    return Detected_ArUco_markers



def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##
	for box_id in Detected_ArUco_markers:
		box = Detected_ArUco_markers[box_id]
		topLeft = int(box[0][0][0]), int(box[0][0][1])
		topRight = int(box[0][1][0]), int(box[0][1][1])
		bottomRight = int(box[0][2][0]), int(box[0][2][1])
		bottomLeft = int(box[0][3][0]), int(box[0][3][1])

		center_x = int((topLeft[0] + bottomRight[0]) / 2)
		center_y = int((topLeft[1] + bottomRight[1]) / 2)
		
		top_middle_x = int((topLeft[0] + topRight[0]) / 2)
		top_middle_y = int((topLeft[1] + topRight[1]) / 2)

		slope =  abs(center_y - top_middle_y) / abs(top_middle_x - center_x)
		radians = math.atan(slope)
		angle = int(math.degrees(radians))

		if((topRight[0] > topLeft[0]) & (topLeft[1] < topRight[1])):
			angle = angle
		elif((topRight[0] > topLeft[0]) & (topLeft[1] == topRight[1])):
			angle = 90
		elif((topRight[0] == topLeft[0]) & (topLeft[1] > topRight[1])):
			angle = 180
		elif((topRight[0] > topLeft[0]) & (topLeft[1] > topRight[1])):
			angle = 180 - angle
		elif((topRight[0] < topLeft[0]) & (topLeft[1] == topRight[1])):
			angle = 270
		elif((topRight[0] < topLeft[0]) & (topLeft[1] > topRight[1])):
			angle = 180 + angle
		elif((topRight[0] < topLeft[0]) & (topLeft[1] < topRight[1])):
			angle = 360 - angle
		elif((topRight[0] == topLeft[0]) & (topLeft[1] < topRight[1])):
			angle = 0

		ArUco_marker_angles[box_id] = angle
		
	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
	GRAY = (125,125,125)
	GREEN = (0,255,0)
	PINK = (180,105,255)
	WHITE = (255,255,255)
	RED = (0,0,255)
	BLUE = (255,0,0)
	BLACK = (0,0,0)

	for box_id in Detected_ArUco_markers:
		box = Detected_ArUco_markers[box_id]
		topLeft = int(box[0][0][0]), int(box[0][0][1])
		topRight = int(box[0][1][0]), int(box[0][1][1])
		bottomRight = int(box[0][2][0]), int(box[0][2][1])
		bottomLeft = int(box[0][3][0]), int(box[0][3][1])

		center_x = int((topLeft[0] + bottomRight[0]) / 2)
		center_y = int((topLeft[1] + bottomRight[1]) / 2)
		
		top_middle_x = int((topLeft[0] + topRight[0]) / 2)
		top_middle_y = int((topLeft[1] + topRight[1]) / 2)

		cv.line(img, topLeft, topRight, BLACK,2)
		cv.line(img, topRight, bottomRight, BLACK,2)
		cv.line(img, bottomRight, bottomLeft, BLACK,2)
		cv.line(img, bottomLeft, topLeft, BLACK,2)
		cv.line(img,(center_x,center_y),(top_middle_x,top_middle_y),BLUE,2)
		cv.circle(img, topLeft, 5, GRAY,-1)
		cv.circle(img, topRight, 5, GREEN,-1)
		cv.circle(img, bottomRight, 5, PINK,-1)
		cv.circle(img, bottomLeft, 5, WHITE,-1)
		cv.circle(img, (center_x,center_y), 5, RED,-1)
		cv.putText(img, str(box_id), (center_x + 20,center_y), cv.FONT_HERSHEY_PLAIN, 3, RED,2)
		cv.putText(img, str(ArUco_marker_angles[box_id]), (center_x - 100,center_y), cv.FONT_HERSHEY_PLAIN, 3, GREEN,2)

	return img


