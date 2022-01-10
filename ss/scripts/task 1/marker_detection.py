#!/usr/bin/env python3


'''
e-Yantra Robotics Challenge
TASK 1.2


Team SS#1377
Team Members:
	Rohit Kumar
	Rohit Vamsi
	Roshan Mallikarjun
	Nidhish Zanwar

Source code for ArUco marker detection using OpenCV and ROS Noetic

This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''


from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from SS_1377_aruco_library import *


PUBLISH_FREQUENCY = 10

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        # Subscribing to /camera/camera/image_raw
		
		rate = rospy.Rate(PUBLISH_FREQUENCY)  #Rate object for publishing to /marker_markerInfo topic. This object is used as a parameter for callback function.
		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback, rate) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
	# Callback function of amera topic
	def image_callback(self, data, rate):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			img, markerInfo = self.ProcessImage()

			#iterate over all the markers in markerInfo
			for marker in markerInfo:
				self.FillMarkerMsg(marker)  #fill the marker_msg variable of the image_proc class.
				self.publish_data(rate)     # function to publish the data at 10 Hz.
		except CvBridgeError as e:
			print(e)
			return

	#  The process image method is used to process the image data from /camera/camera/image_raw topic and uses aruco_library.py from task 1.1
	#  mark_aruco function is modified to return a tuple of the image with marked aruco markers and a list of lists that contain information
	#  pertaining to the various aruco markers in the camera feed.
	def ProcessImage(self):
		Detected_ArUco_markers = detect_ArUco(self.img)					
		angle = Calculate_orientation_in_degree(Detected_ArUco_markers)			
		img, info = mark_ArUco(self.img,Detected_ArUco_markers,angle)  
		return img, info											  

	#The FillMarkerMsg method takes a list of information of a marker and fills marker_msg of Marker data type
	def FillMarkerMsg(self, markerInfo):
		self.marker_msg.id = markerInfo[0]
		self.marker_msg.x = markerInfo[1]
		self.marker_msg.y = markerInfo[2]
		self.marker_msg.z = markerInfo[3]
		self.marker_msg.roll = markerInfo[4]
		self.marker_msg.pitch = markerInfo[5]
		self.marker_msg.yaw = markerInfo[6]
		return self.marker_msg
			 
	# publishes the data at 10 Hz frequency.
	def publish_data(self, rate):
		self.marker_pub.publish(self.marker_msg)
		rate.sleep()
		

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
