#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_markerInfo
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from aruco_library import *

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_markerInfo', Marker, queue_size=1)
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw
		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
	# Callback function of amera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			img, markerInfo = self.ProcessImage()
			self.FillMarkerMsg(markerInfo)
			self.publish_data()
		except CvBridgeError as e:
			print(e)
			return

	def ProcessImage(self):
		Detected_ArUco_markers = detect_ArUco(self.img)									## detecting ArUco ids and returning ArUco dictionary
		angle = Calculate_orientation_in_degree(Detected_ArUco_markers)				## finding orientation of aruco with respective to the menitoned scale in problem statement
		img, info = mark_ArUco(self.img,Detected_ArUco_markers,angle)
		return img, info

	def FillMarkerMsg(self, markerInfo):
		self.marker_msg.id = markerInfo[0]
		self.marker_msg.x = markerInfo[1]
		self.marker_msg.y = markerInfo[2]
		self.marker_msg.z = markerInfo[3]
		self.marker_msg.roll = markerInfo[4]
		self.marker_msg.pitch = markerInfo[5]
		self.marker_msg.yaw = markerInfo[6]
			
	def publish_data(self):
		print(type(self.marker_msg))
		#self.marker_pub.publish(self.marker_msg)
		

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
