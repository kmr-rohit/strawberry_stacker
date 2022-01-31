#!/usr/bin/env python3

'''
e-Yantra Robotics Competition
Task 3.2
This python file is team SS#1377 submission for task 3.2 of e-YRC.
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
    /edrone0/mavros/state
    /edrone0/mavros/local_position/pose
    /edrone0/mavros/local_position/velocity
    /edrone1/mavros/state
    /edrone1/mavros/local_position/pose
    /edrone1/mavros/local_position/velocity
    /edrone0/gripper_check
    /edrone1/gripper_check
    /eDrone0/camera/image_raw
    /eDrone1/camera/image_raw
    Publications:
    /edrone0/mavros/setpoint_position/local
    /edrone1/mavros/setpoint_position/local
    /edrone0/mavros/setpoint_velocity/cmd_vel
    /edrone1/mavros/setpoint_velocity/cmd_vel
    Services To Be Called:
    /edrone0/mavros/cmd/arming
    /edrone0/mavros/set_mode
    /edrone1/mavros/cmd/arming
    /edrone1/mavros/set_mode
    /edrone0/mavros/cmd/land
    /edrone1/mavros/cmd/land
    /edrone0/activate_gripper
    /edrone1/activate_gripper
    
'''
import cv2 as cv
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2.aruco as aruco
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper
from std_msgs.msg import *
import time

# Publish setpoints at 200 Hz
PUBLISH_FREQUENCY = 400

class OffboardControl:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    # set arm=true for arming the drone and arm=false for disarming
    def SetArm0(self, arm):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/edrone0/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                '/edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(arm)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def SetArm1(self, arm):
        rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                '/edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(arm)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies
    def OffboardSetMode0(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:
            # Creating a proxy service for the rosservice named /mavros/set_mode for setting the mode
            flightModeService = rospy.ServiceProxy(
                '/edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)
        
    def OffboardSetMode1(self):
        rospy.wait_for_service('/edrone1/mavros/set_mode')
        try:
            # Creating a proxy service for the rosservice named /mavros/set_mode for setting the mode
            flightModeService = rospy.ServiceProxy(
                '/edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)

    def LandDrone0(self):
        rospy.wait_for_service('/edrone0/mavros/cmd/land')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy(
                '/edrone0/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landingService()
        except rospy.ServiceException as e:
            print("service /mavros/cmd/land call failed: %s. Drone could not land." % e)
    def LandDrone1(self):
        rospy.wait_for_service('/edrone1/mavros/cmd/land')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy(
                '/edrone1/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landingService()
        except rospy.ServiceException as e:
            print("service /mavros/cmd/land call failed: %s. Drone could not land." % e)

    def takeoffDrone0(self):
        rospy.wait_for_service('edrone0/mavros/cmd/takeoff')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy('edrone0/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            landingService(altitude=15.0)
        except rospy.ServiceException as e:
            print("service /mavros/cmd/land call failed: %s. Drone could not land." % e)

    def takeoffDrone1(self):
        rospy.wait_for_service('edrone1/mavros/cmd/takeoff')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy('edrone1/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            landingService(altitude=15.0)
        except rospy.ServiceException as e:
            print("service /mavros/cmd/land call failed: %s. Drone could not land." % e)


    # Function for activating and deactivating the gripper.
    # set state=True for activating and state=False for deactivating
    def ActivateGripper0(self, state):
        rospy.wait_for_service('/edrone0/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
            gripperService(state)
        except rospy.ServiceException as e:
            print(
                "service activate_gripper call failed: %s. gripper could not be activated." % e)
    def ActivateGripper1(self, state):
        rospy.wait_for_service('/edrone1/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/edrone1/activate_gripper', Gripper)
            gripperService(state)
        except rospy.ServiceException as e:
            print(
                "service activate_gripper call failed: %s. gripper could not be activated." % e)



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
        current_pos = np.array(
            (self.droneInfo.position.x, self.droneInfo.position.y, self.droneInfo.position.z))
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
        self.id = 0
        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.center = (0, 0)
        self.box_x = 0

    def FindCenter(self, box):
        topLeft = int(box[0][0][0]), int(box[0][0][1])
        bottomRight = int(box[0][2][0]), int(box[0][2][1])

        # coordinates of the center of the bounding box of aruco marker
        center_x = int((topLeft[0] + bottomRight[0]) / 2)
        center_y = int((topLeft[1] + bottomRight[1]) / 2)

        self.center = (center_x, center_y)

    def ProcessImage(self):
        imgGray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # returns a list bounding boxes of aruco markers and a list of their respective ids
        bboxes, ids, rejected = aruco.detectMarkers(imgGray, arucoDict)
        if len(bboxes) != 0:  # check if any aruco markers are detected
            self.FindCenter(bboxes[0])
            self.id = ids[0]
            return True
        return False

    def ArucoCallback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detectedAruco = self.ProcessImage()

        except CvBridgeError as e:
            print(e)
            return


#Calculation Formula From: https://answers.opencv.org/question/56744/converting-pixel-displacement-into-other-unit/
def CalculateOffsetPosAruco(coord):
    #alpha value is calculated from horizontal FOV and dimensions of the image. HFOV = 80 deg, imag res = 400x400
    alpha = 0.2 # alpha is the angular resolution in deg / pixel.
    x = coord[0]
    y = coord[1]
    Dc = math.sqrt(pow((x - 200), 2) + pow((y - 200), 2))
    angle = math.radians(Dc * alpha)
    Dw = 3 * math.sin(angle)
    return Dw

def SendDroneToSetpoint(setPoint, rate, stateMonitor, localPosPublisher, offboardControl, arucoDetect, scan):
    offset = 0.3
    pos = PoseStamped()
    pos.pose.position.x = setPoint[0]
    pos.pose.position.y = setPoint[1]
    pos.pose.position.z = setPoint[2]

    setPt = np.array(setPoint)
    while True:
        print("Sending Drone to setpoint", end=' ')
        print(setPoint)
        if stateMonitor.ReachedSetpoint(setPt, offset):
            # Wait 2 seconds for the drone to settle in the setpoint.
            time.sleep(2.0)
            break
        if scan:
            if arucoDetect.detectedAruco:
                x_coord = stateMonitor.droneInfo.position.x
                y_coord = stateMonitor.droneInfo.position.y
                print(x_coord, y_coord)
                print("Aruco with aruco id " +
                      str(arucoDetect.id) + " Detected")
                print(arucoDetect.center)
                arucoDetect.box_x = x_coord + \
                    CalculateOffsetPosAruco(arucoDetect.center) + 1.15
                newSetPoint = [arucoDetect.box_x, 0, 3]
                SendDroneToSetpoint(newSetPoint, rate, stateMonitor,
                                    localPosPublisher, offboardControl, arucoDetect, False)
                

                break
        localPosPublisher.publish(pos)
        # rate.sleep()

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
    localPosPublisher = rospy.Publisher(
        '/edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    localPosPublisher = rospy.Publisher(
        '/edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # # Setpoint variable topic
    localVelPublisher = rospy.Publisher(
        '/edrone0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    
    localVelPublisher = rospy.Publisher(
        '/edrone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # Initialize subscribers

    rospy.Subscriber("/edrone0/mavros/state", State, stateMonitor.StateCallback)
    rospy.Subscriber("/edrone1/mavros/state", State, stateMonitor.StateCallback)

    # subscriber for local position
    rospy.Subscriber("/edrone0/mavros/local_position/pose",
                     PoseStamped, stateMonitor.LocalPosCallback)
    rospy.Subscriber("/edrone1/mavros/local_position/pose",
                     PoseStamped, stateMonitor.LocalPosCallback)

    # subscriber for local velocity
    rospy.Subscriber("/edrone0/mavros/local_position/velocity",
                     TwistStamped, stateMonitor.LocalVelCallback)
    rospy.Subscriber("/edrone1/mavros/local_position/velocity",
                     TwistStamped, stateMonitor.LocalVelCallback)

    # subscriber for gripper state
    rospy.Subscriber('/edrone0/gripper_check', String, stateMonitor.GripperCallback)
    rospy.Subscriber('/edrone1/gripper_check', String, stateMonitor.GripperCallback)

    # Subscribing to the camera topic
    rospy.Subscriber("/eDrone/camera/image_raw",
                     Image, arucoDetect.ArucoCallback)

    # Rate variable to publish at PUBLISH_FREQUENCY
    rate = rospy.Rate(PUBLISH_FREQUENCY)

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
        offboardControl.SetArm0(True)  # Call the arming service
        print("Arming the drone0")
        offboardControl.SetArm1(True)  # Call the arming service
        print("Arming the drone1")
        rate.sleep()   
    

    # Switching the state to OFFBOARD mode
    while not stateMonitor.state.mode == "OFFBOARD":
        print("setting to offboard")
        offboardControl.OffboardSetMode0()
        # Call the offboard mode service
        offboardControl.OffboardSetMode1() 
           # Call the offboard mode service 
        rate.sleep()
    print("OFFBOARD mode activated")



         

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass