#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''
import numpy as np
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    #set arm=true for arming the drone and arm=false for disarming
    def setArm(self, arm):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(arm)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies
    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            # Creating a proxy service for the rosservice named /mavros/set_mode for setting the mode
            flightModeService = rospy.ServiceProxy(
                'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)

    def landDrone(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/land for landing
            landingService = rospy.ServiceProxy(
                '/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landingService()
        except rospy.ServiceException as e:
            print("service /mavros/cmd/land call failed: %s. Drone could not land." % e)

class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)

        # LOCAL_NED
        self.sp.coordinate_frame = 1

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # # Create more callback functions for other subscribers

    def local_posCb(self, msg):
        # Callback function for topic /mavros/local_position/pose
        self.sp.position.x = msg.pose.position.x
        self.sp.position.y = msg.pose.position.y
        self.sp.position.z = msg.pose.position.z

    def local_velCb(self, msg):
        # Callback function for topic /mavros/local_position/velocity
        self.sp.velocity.x = msg.twist.linear.x
        self.sp.velocity.y = msg.twist.linear.y
        self.sp.velocity.z = msg.twist.linear.z

    def reached_setpoint(self, setpt, offset):
        current_pos = np.array((self.sp.position.x, self.sp.position.y, self.sp.position.z))
        return np.linalg.norm(setpt - current_pos) <= offset


def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(140.0)

    # Make the list of setpoints
    setpoints = [[0, 0, 10], [10, 0, 10], [10, 10, 10], [0, 10, 10], [0, 0, 10]]  # List to setpoints

    # Similarly initialize other publishers

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

    # Similarly add other containers

    # Initialize subscriber
    rospy.Subscriber("mavros/state", State, stateMt.stateCb)

    # Similarly initialize other subscribers

    # subscriber for local position
    rospy.Subscriber("mavros/local_position/pose",
                     PoseStamped, stateMt.local_posCb)

    # subscriber for local velocity
    rospy.Subscriber("mavros/local_position/velocity",
                     TwistStamped, stateMt.local_velCb)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    # Send some dummy setpoints
    for i in range(100):
        print("Sending dummy setpoints")
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm(True)  # Call the arming service
        rate.sleep()
    print("Armed!!")

    # Switching the state to OFFBOARD mode
    while not stateMt.state.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode()    # Call the offboard mode service
        rate.sleep()
    print("OFFBOARD mode activated")

    # Publish the setpoints
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  
        Write your algorithm here 
        '''
        # Step 1: Set the setpoint
        for i in range(len(setpoints)):
            
            pos.pose.position.x = setpoints[i][0]
            pos.pose.position.y = setpoints[i][1]
            pos.pose.position.z = setpoints[i][2]
            stpt = np.array((setpoints[i][0], setpoints[i][1], setpoints[i][2]))
            print("Sending setpoint " + str(i))
            offset = 0.5
            local_pos_pub.publish(pos)
            # Step 2: Then wait till the drone reaches the setpoint,
            while True:
                if stateMt.reached_setpoint(stpt, offset):
                    break
        #land the drone
        ofb_ctl.landDrone()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
