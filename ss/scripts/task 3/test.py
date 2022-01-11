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
    /mavros/setpoint_position/local
    /mavros/setpoint_velocity/cmd_vel

    Services To Be Called:
    /mavros/cmd/arming
    /mavros/cmd/land
    /mavros/set_mode
    /activate_gripper
    
'''
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper
from std_msgs.msg import *
import time

#Publish setpoints at 200 Hz
PUBLISH_FREQUENCY = 200

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
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)

    def LandDrone(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landingService()
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
    
    # Function to wait for the drone to land
    def WaitForLanding(self):
        while True:
            print("Landing")
            if self.droneInfo.position.z < 0:
                break
        time.sleep(2.0)
        print("LANDED")


class ArucoDetectInfo:
    def __init__(self):
        self.detectedAruco = False
        self.img = np.empty([]) # This will contain your image frame from camera
        self.bridge = CvBridge()

    def ProcessImage(self):
        imgGray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        bboxes, ids, rejected = aruco.detectMarkers(imgGray,arucoDict)  # returns a list bounding boxes of aruco markers and a list of their respective ids

        if len(bboxes) != 0:     #check if any aruco markers are detected
            return True;
        return False;

    def ArucoCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            self.detectedAruco = self.ProcessImage()

        except CvBridgeError as e:
            print(e)
            return


# Function to publish the setpoint coordinates in localPosPublisher channel at rate frequency
# This function also waits until the drone reaches the setpoint.
def SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher, offboardControl, arucoDetect, scan):

    offset = 0.3
    pos = PoseStamped()
    pos.pose.position.x = setPoint[0]
    pos.pose.position.y = setPoint[1]
    pos.pose.position.z = setPoint[2]

    setPt = np.array(setPoint)
    while True:
        if stateMonitor.ReachedSetpoint(setPt, offset):
            time.sleep(2.0)   # Wait 2 seconds for the drone to settle in the setpoint.
            break
        if scan:
            if arucoDetect.detectedAruco:
                offboardControl.LandDrone()
                stateMonitor.WaitForLanding()
                if stateMonitor.gripperCheck == 'True':
                    offboardControl.ActivateGripper(True)
                    time.sleep(2.0) # Wait for the drone to activate the gripper
            # newSetPoint = [stateMonitor.droneInfo.position.x, stateMonitor.droneInfo.position.y, 3]
            # SendDroneToSetpoint(newSetPoint, rate, stateMonitor, localPosPublisher, offboardControl, arucoDetect, scan)

        localPosPublisher.publish(pos)
        rate.sleep()

    print(f"Reached setpoint {setPoint}")

def main():

    #PUBLISH_FREQUENCY = int(input('Enter frequency: '))

    # StateMonitor instance to keep track of the drone information
    stateMonitor = StateMonitor()

    # OffboardControl instance to control the drone in offboard mode
    offboardControl = OffboardControl()

    arucoDetect = ArucoDetectInfo()

    # Initialize publishers

    # Setpoint position topic
    localPosPublisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Setpoint variable topic
    localVelPublisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    
    # Initialize subscribers

    rospy.Subscriber("/mavros/state", State, stateMonitor.StateCallback)

    # subscriber for local position
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, stateMonitor.LocalPosCallback)

    # subscriber for local velocity
    rospy.Subscriber("/mavros/local_position/velocity",TwistStamped, stateMonitor.LocalVelCallback)

    # subscriber for gripper state
    rospy.Subscriber('/gripper_check', String, stateMonitor.GripperCallback)

    rospy.Subscriber("/eDrone/camera/image_raw", Image, arucoDetect.ArucoCallback) #Subscribing to the camera topic

    # Rate variable to publish at PUBLISH_FREQUENCY 
    rate = rospy.Rate(PUBLISH_FREQUENCY)

    # Setpoints to be covered by the drone
    setPoints = [[0, 0, 3], [9, 0, 3]]

    scanPoint = [9, 0, 3]

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 1.0
    vel.twist.linear.y = 1.0
    vel.twist.linear.z = 1.0

    # Publish dummy setpoints
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

        # Iterate over all the setpoints
        for setPoint in setPoints:

            print(setPoint)
            SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher, offboardControl, arucoDetect, False)
            if setPoint == scanPoint:
                SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher, offboardControl, arucoDetect, True)

        offboardControl.LandDrone()
        break
                    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# # Land the drone at boxPickup and activate gripper if the box is in the allowed range
#             if(setPoint == boxPickup):
#                 while not boxPickedUp:
#                     SendDroneToSetpoint([setPoint[0], setPoint[1], -0.8], rate, stateMonitor, localPosPublisher)
#                     stateMonitor.WaitForLanding()
#                     time.sleep(1.0)  # Wait for the drone to settle
#                     if stateMonitor.gripperCheck.data == "True":
#                         print("Correct gripper position")
#                         offboardControl.ActivateGripper(True)
#                         time.sleep(2.0) # Wait for the drone to activate the gripper
#                         boxPickedUp = True

#                     SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher)

#             #Land at the boxDropOff point and drop the box
#             if(setPoint == boxDropOff):
#                 SendDroneToSetpoint([setPoint[0], setPoint[1], -0.8], rate, stateMonitor, localPosPublisher)
#                 stateMonitor.WaitForLanding()
#                 time.sleep(2.0) # Wait for the drone to settle
#                 offboardControl.ActivateGripper(False)  # Deactivate the gripper
#                 print("Box Dropped at drop off point")
#                 SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher)
