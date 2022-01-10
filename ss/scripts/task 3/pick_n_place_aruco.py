#!/usr/bin/env python3

'''
e-Yantra Robotics Competition
Task 3.1

This python file is team SS#1377 submission for task 3.1 of e-YRC.
Team SS 1377
Team members:
    Rohit Kumar
    Rohit Bacharaju
    Nidhish Zanwar
    Roshan Mallikarjun

This python file runs a ROS-node of name offboard_control which controls the drone to pick up, drop and 
navigate with the box in offboard mode. 
This node publishes and subsribes the following topics:

    Subscriptions:
    /mavros/state
    /mavros/local_position/pose
    /gripper_check

    Publications:
    /mavros/setpoin_position/local
    /mavros/setpoint_velocity/cmd_vel

    Services To Be Called:
    
'''
import rospy
import numpy as np
import math
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper
from std_msgs.msg import *
from task_1 import *



class OffboardControl:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)
        

    #set arm=true for arming the drone and arm=false for disarming
    def SetArm(self, arm):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(arm)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies
    def OffboardSetMode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('/mavros/set_mode')
        try:
            # Creating a proxy service for the rosservice named /mavros/set_mode for setting the mode
            setModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service set_mode call failed: %s" % e)
            
    def landDrone(self):
        # Call /mavros/set_mode to set the mode the drone to LAND
        # and print fail message on failure
        rospy.wait_for_service('/mavros/set_mode')
        try:
            # Creating a proxy service for the rosservice named /mavros/set_mode for setting the mode
            setModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode='LAND')
        except rospy.ServiceException as e:
            print("Service set_mode call failed: %s" % e)

    def takeoffDrone(self):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            landingService(altitude=15.0)
        except rospy.ServiceException as e:
            print("service /mavros/cmd/land call failed: %s. Drone could not land." % e)

    #Function for activating and deactivating the gripper.
    #set state=True for activating and state=False for deactivating
    def ActivateGripper(self, state):
        rospy.wait_for_service('/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/activate_gripper', Gripper)
            gripperService(state)
        except rospy.ServiceException as e:
            print("service activate_gripper call failed: %s. gripper could not be activated." % e)

class aruco_detection: 
    
    
    def __init__(self):
        #initialise everything and set the rate
        rate = rospy.Rate(10)
        bridge = CvBridge()
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters =  cv2.aruco.DetectorParameters_create()
        #initialise the node
        rospy.init_node('aruco_detection', anonymous=True)
        #initialise the subscribers and publishers
        self.image_sub = rospy.Subscriber("/eDrone/camera/image_raw", Image, self.callback)
        self.aruco_pub = rospy.Publisher('aruco_pose', PoseStamped, queue_size=10)
        self.aruco_pose = PoseStamped()

    def callback(self, data):
        #convert the image to a numpy array
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #detect the aruco marker
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        #if the marker is detected
        if ids is not None:
            #estimate the pose of the aruco marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, np.eye(3, dtype=np.float32), self.camera_matrix, self.dist_coeffs)
            #draw the aruco marker
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            #draw the aruco axis
            cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
            #convert the cv_image to a numpy array
            cv_image = np.array(cv_image)
            #convert the cv_image to a ROS image
            self.aruco_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            #publish the aruco image
            self.aruco_pub.publish(self.aruco_image)
    

            
        
    


class StateMonitor:

    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.droneInfo = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.droneInfo.type_mask = int('010111111000', 2)

        # LOCAL_NED
        self.droneInfo.coordinate_frame = 1

        self.gripperCheck = String()

    def StateCallback(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # # Create more callback functions for other subscribers

    def LocalPosCallback(self, msg):
        # Callback function for topic /mavros/local_position/pose
        self.droneInfo.position.x = msg.pose.position.x
        self.droneInfo.position.y = msg.pose.position.y
        self.droneInfo.position.z = msg.pose.position.z

    def LocalVelCallback(self, msg):
        # Callback function for topic /mavros/local_position/velocity
        self.droneInfo.velocity.x = msg.twist.linear.x
        self.droneInfo.velocity.y = msg.twist.linear.y
        self.droneInfo.velocity.z = msg.twist.linear.z

    def ReachedSetpoint(self, setpt, offset):
        current_pos = np.array((self.droneInfo.position.x, self.droneInfo.position.y, self.droneInfo.position.z))
        return np.linalg.norm(setpt - current_pos) <= offset

    def GripperCallback(self, msg):
        self.gripperCheck = msg

    def WaitForLanding(self):
        while True:
            if self.droneInfo.position.z < 0.62:
                break
        time.sleep(2.0)
        print("Landed")


def SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher):

    offset = 0.3
    pos = PoseStamped()
    pos.pose.position.x = setPoint[0]
    pos.pose.position.y = setPoint[1]
    pos.pose.position.z = setPoint[2]

    setPt = np.array(setPoint)
    while True:
        if stateMonitor.ReachedSetpoint(setPt, offset):
            time.sleep(2.0)
            break
        localPosPublisher.publish(pos)
        rate.sleep()
    print(f"Reached setpoint {setPoint}")

def landDroneOnBox(setPoint, localPosPublisher, rate):
    currentTime = time.time()
    pos = PoseStamped()
    pos.pose.position.x = setPoint[0]
    pos.pose.position.y = setPoint[1]
    pos.pose.position.z = setPoint[2]
    while True:
        if time.time() - currentTime > 15:
            break
        localPosPublisher.publish(pos)
        rate.sleep()
    print("Landed")
    



def main():

    stateMonitor = StateMonitor()
    offboardControl = OffboardControl()
    


    # Initialize publishers
    localPosPublisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    localVelPublisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # Initialize subscribers
    rospy.Subscriber("/mavros/state", State, stateMonitor.StateCallback)
    # subscriber for local position
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, stateMonitor.LocalPosCallback)
    # subscriber for local velocity
    rospy.Subscriber("/mavros/local_position/velocity",TwistStamped, stateMonitor.LocalVelCallback)
    # subscriber for gripper state
    rospy.Subscriber('/gripper_check', String, stateMonitor.GripperCallback)


    rate = rospy.Rate(140.0)

    setPoints = [[0 ,0 , 3], [6,0,3], [9,0,3]]
    boxPickup = [6,0,3]
    boxDropOff = [9,0,3]

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 5.0
    vel.twist.linear.y = 5.0
    vel.twist.linear.z = 5.0

    for i in range(100):
        print("Sending dummy setpoints")
        localPosPublisher.publish(pos)
        rate.sleep()

    # Arming the drone
    while not stateMonitor.state.armed:
        offboardControl.SetArm(True)  # Call the arming service
        rate.sleep()
    print("Armed!!")

    # Switching the state to OFFBOARD mode
    while not stateMonitor.state.mode == "OFFBOARD":
        print(stateMonitor.state.mode)
        print("setting to offboard")
        offboardControl.OffboardSetMode()    # Call the offboard mode service
        rate.sleep()
    print("OFFBOARD mode activated")


    while not rospy.is_shutdown():

        boxPickedUp = False
        boxDropped = False

        for setPoint in setPoints:

            print(setPoint)
            SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher)
            if(setPoint == boxPickup):
                while not boxPickedUp:
                    SendDroneToSetpoint([setPoint[0], setPoint[1], -0.6], rate, stateMonitor, localPosPublisher)
                    stateMonitor.WaitForLanding()
                    time.sleep(2.0)
                    if stateMonitor.gripperCheck.data == "True":
                        print("Correct gripper position")
                        offboardControl.ActivateGripper(True)
                        time.sleep(3)
                        boxPickedUp = True
                    SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher)

            if(setPoint == boxDropOff):
                SendDroneToSetpoint([setPoint[0], setPoint[1], -0.6], rate, stateMonitor, localPosPublisher)
                stateMonitor.WaitForLanding()
                time.sleep(2.0)
                offboardControl.ActivateGripper(False)
                SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher)

        offboardControl.landDrone()
                    

                    
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

        
