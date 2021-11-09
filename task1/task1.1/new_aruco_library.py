#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

'''
e-Yantra Robotics Challenge
Team SS#1377
Team Members:
	Rohit Kumar
	Rohit Vamsi
	Roshan Mallikarjun
	Nidhish Zanwar
Source code for ArUco marker detection using OpenCV
'''

import numpy as np
import cv2 as cv
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
    arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    bboxes, ids, rejected, = aruco.detectMarkers(imgGray,arucoDict)  # returns a list bounding boxes of aruco markers and a list of their respective ids

    if len(bboxes) != 0:     #check if any aruco markers are detected
        for bbox, box_id in zip(bboxes,ids):
    	    Detected_ArUco_markers[box_id[0]] = bbox

	#  returns a dictionary where the keys are aruco ids and the values are the list 
	#  of corner points of bounding box of aruco id		
    return Detected_ArUco_markers 



def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##
	# iterates over the dictionary returned in detect_aruco function
	for box_id in Detected_ArUco_markers:
		box = Detected_ArUco_markers[box_id]

		# coordinates of 3 corner points of bounding box of the aruco marker
		topLeft = int(box[0][0][0]), int(box[0][0][1])
		topRight = int(box[0][1][0]), int(box[0][1][1])
		bottomRight = int(box[0][2][0]), int(box[0][2][1])

		# coordinates of the center of the bounding box of aruco marker
		center_x = int((topLeft[0] + bottomRight[0]) / 2)
		center_y = int((topLeft[1] + bottomRight[1]) / 2)
		
		# coordinates of the midpoint of top edge of bounding box of aruco marker
		top_middle_x = int((topLeft[0] + topRight[0]) / 2)
		top_middle_y = int((topLeft[1] + topRight[1]) / 2)

		#finding orientation of the aruco marker
		
		angle = 0
		if top_middle_x != center_x:
			slope =  - (top_middle_y - center_y) / (top_middle_x - center_x)
			radians = math.atan(slope)
			angle = int(math.degrees(radians))

			if top_middle_x < center_x:
				angle = 180 + angle

			elif top_middle_x > center_x and top_middle_y > center_y:
				angle = 360 + angle

		elif top_middle_x == center_x:
			if top_middle_y < center_y:
				angle = 90
			elif top_middle_y > center_y:
				angle = 270

		ArUco_marker_angles[box_id] = angle
		
	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##

	#COLOR           B    G    R
	GRAY        = (125, 125, 125)
	GREEN       = (  0, 255,   0)
	PINK        = (180, 105, 255)
	WHITE       = (255, 255, 255)
	RED         = (  0,   0, 255)
	BLUE        = (255,   0,   0)
	BLACK       = (  0,   0,   0)

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

		angle = ArUco_marker_angles[box_id]

		markerInfo = [box_id, center_x, center_y, 0, 0, 0, angle]

		# drawing a black box around the aruco marker
		cv.line(img, topLeft, topRight, BLACK,2)
		cv.line(img, topRight, bottomRight, BLACK,2)
		cv.line(img, bottomRight, bottomLeft, BLACK,2)
		cv.line(img, bottomLeft, topLeft, BLACK,2)

		# drawing a blue line between the center of aruco marker to midpoint of top edge
		cv.line(img,(center_x,center_y),(top_middle_x,top_middle_y),BLUE,2)

		# drawing dots of different colors at the four corners
		cv.circle(img, topLeft, 5, GRAY,-1)
		cv.circle(img, topRight, 5, GREEN,-1)
		cv.circle(img, bottomRight, 5, PINK,-1)
		cv.circle(img, bottomLeft, 5, WHITE,-1)
		cv.circle(img, (center_x,center_y), 5, RED,-1)

		# writing the angle of orientation and id of the aruco marker
		cv.putText(img, str(box_id), (center_x + 20,center_y), cv.FONT_HERSHEY_PLAIN, 2, RED,2)
		cv.putText(img, str(angle), (center_x - 100,center_y), cv.FONT_HERSHEY_PLAIN, 2, GREEN,2)

	return img, markerInfo

'''
for testing:
while True:
	ret, frame = cap.read()
	if cv.waitKey(1) & 0xFF == 27:
		break
	Detected_ArUco_markers = detect_ArUco(frame)									## detecting ArUco ids and returning ArUco dictionary
	angle = Calculate_orientation_in_degree(Detected_ArUco_markers)				## finding orientation of aruco with respective to the menitoned scale in problem statement
	img = mark_ArUco(frame,Detected_ArUco_markers,angle)						## marking the parameters of aruco which are mentioned in the problem statement
	cv.imshow(img)
'''
