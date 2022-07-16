#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:


    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)


    
    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

   
    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService=rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print( 'Service set_mode call failed: %s. Offboard Mode could not be set.',e )

    def autoland_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService=rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("Service autoland call failed: %s",e)


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message

        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    


def main():


    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [] #List to setpoints

    pos1=PoseStamped()
    pos1.pose.position.x = 0
    pos1.pose.position.y = 0
    pos1.pose.position.z = 10

    vel1=Twist()
    vel1.linear.x=5

    setpoints.append((pos1,vel1))
    
    pos2=PoseStamped()
    pos2.pose.position.x = 10
    pos2.pose.position.y = 0
    pos2.pose.position.z = 10

    vel2=Twist()
    vel2.linear.x=5

    setpoints.append((pos2,vel2))

    pos3=PoseStamped()
    pos3.pose.position.x = 10
    pos3.pose.position.y = 10
    pos3.pose.position.z = 10

    vel3=Twist()
    vel3.linear.x=5

    setpoints.append((pos3,vel3))

    pos4=PoseStamped()
    pos4.pose.position.x = 0
    pos4.pose.position.y = 10
    pos4.pose.position.z = 10

    vel4=Twist()
    vel4.linear.x=5

    setpoints.append((pos4,vel4))

    pos5=PoseStamped()
    pos5.pose.position.x = 0
    pos5.pose.position.y = 0
    pos5.pose.position.z = 10

    vel5=Twist()
    vel5.linear.x=5

    setpoints.append((pos5,vel5))


    # Similarly initialize other publishers 

    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    
    # Similarly add other containers

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 


    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()


    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    # while not stateMt.state.mode=="OFFBOARD":
        # ofb_ctl.offboard_set_mode()
        # rate.sleep()
    ofb_ctl.offboard_set_mode()
    print ("OFFBOARD mode activated")

    # Publish the setpoints 
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        i=0
        while not rospy.is_shutdown():
            if (i<len(setpoints)):
                for _ in range(100):
                    local_pos_pub.publish(setpoints[i][0])
                    local_vel_pub.publish(setpoints[i][1])
                    rate.sleep()
                i=i+1
            else:
                ofb_ctl.autoland_set_mode()
                break


    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
