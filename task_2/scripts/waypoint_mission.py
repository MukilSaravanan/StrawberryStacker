#!/usr/bin/env python3
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Modes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print( "Service arming call failed: %s",e)

    def auto_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s",e)

    def wpPush(self,index,wps):
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
            wpPushService(start_index=0,waypoints=wps)#start_index = the index at which we want the mission to start
            print ("Waypoint Pushed")
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s",e)
    def wpPull(self,wps):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            print (wpPullService().wp_received)

            print ("Waypoint Pulled")
        except rospy.ServiceException as e:
            print ("Service Puling call failed: %s",e)

class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        
    def stateCb(self, msg):
        self.state = msg

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        self.wp.command = command #VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg"""
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        self.wp.param1=param1 # no idea what these are for but the script will work so go ahead
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt #relative altitude.

        return self.wp


def main():
    rospy.init_node('waypointMission', anonymous=True)
    rate = rospy.Rate(20.0)

    stateMt = stateMoniter()
    md = Modes()
    
    wayp0 = wpMissionCnt()
    wayp1 = wpMissionCnt()
    wayp2 = wpMissionCnt()
    wayp3 = wpMissionCnt()
    wayp4 = wpMissionCnt()
    wayp5 = wpMissionCnt()

    wps = [] #List to story waypoints
    
    w = wayp0.setWaypoints(3,84,True,True,0.0,0.0,0.0,float('nan'),19.134423,72.9117629,10)
    wps.append(w)

    w = wayp1.setWaypoints(3,16,True,True,0.0,0.0,0.0,float('nan'),19.134641,72.911706,10)
    wps.append(w)

    w = wayp2.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134641,72.911710,10)
    wps.append(w)

    w = wayp3.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134434,72.911817,10)
    wps.append(w)

    w = wayp4.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,10)
    wps.append(w)

    w = wayp5.setWaypoints(3,85,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,0)
    wps.append(w)



    print (wps)
    md.wpPush(0,wps)

    md.wpPull(0)
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
    # Switching the state to auto mode
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode()
        rate.sleep()
        print ("AUTO.MISSION")

    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
