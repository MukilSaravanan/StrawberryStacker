#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from std_msgs.msg import String
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper


class offboard_control:

    def __init__(self):
        rospy.init_node('pick_n_place')

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService=rospy.ServiceProxy('mavros/cmd/arming',mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print('Service arm call failed: %s',e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService=rospy.ServiceProxy('mavros/cmd/arming',mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print('Service disarm call failed: %s',e) 

    def offboard_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService=rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print('Service set_mode call failed: %s. Offboard mode could not be set.',e)

    def autoland_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService=rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print('Service autoland call failed: %s',e)


class stateMoniter:
    def __init__(self):
        self.state=State()
    
    def stateCb(self,msg):
        self.state=msg

class psMoniter:
    def __init__(self):
        self.ps=PoseStamped()
        self.del_x=0
        self.del_y=0
        self.del_z=0

    def psCb(self,msg):
        self.ps=msg
        # rospy.loginfo("Inside psCb")

    def is_reached(self,other,thresh):
        # rospy.loginfo("Inside is_reached")
        self.del_x=abs(other.pose.position.x - self.ps.pose.position.x)
        self.del_y=abs(other.pose.position.y - self.ps.pose.position.y)
        self.del_z=abs(other.pose.position.z - self.ps.pose.position.z)
        return (self.del_x<=thresh and self.del_y<=thresh and self.del_z<=thresh)

class gripperMoniter:
    def __init__(self):
        self.status="False"

    def gripperCb(self,msg):
        self.status=msg.data

    def gripper_action(self,action):
        rospy.wait_for_service('/activate_gripper')
        try:
            gripperActionService=rospy.ServiceProxy('/activate_gripper',Gripper)
            status_fromService=gripperActionService(activate_gripper=action)
            rospy.loginfo('status_fromService: {}'.format(status_fromService))
            return status_fromService.result
        except rospy.ServiceException as e:
            print('Service gripper action call failed: %s',e)

    def gripper_status(self):
        pass
def main():
    
    stateMt=stateMoniter()
    ofb_ctl=offboard_control()
    psMt=psMoniter()
    gripperMt=gripperMoniter()

    local_pos_pub=rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)
    local_vel_pub=rospy.Publisher('mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)

    rate=rospy.Rate(30.0)

    setpoints=[]

    pos1=PoseStamped()
    pos1.pose.position.x=0
    pos1.pose.position.y=0
    pos1.pose.position.z=3

    vel1=Twist()

    setpoints.append(pos1)

    pos2=PoseStamped()
    pos2.pose.position.x=3.060673713684082
    pos2.pose.position.y=-0.174725741147995
    pos2.pose.position.z=3

    setpoints.append(pos2)

    pos3=PoseStamped()
    pos3.pose.position.x= 3.060673713684082
    pos3.pose.position.y=-0.174725741147995
    pos3.pose.position.z=-0.5674432516098022

    # pos3=PoseStamped()
    # pos3.pose.position.x=3
    # pos3.pose.position.y=0
    # pos3.pose.position.z=0


    setpoints.append(pos3)

    pos4=PoseStamped()
    pos4.pose.position.x=3
    pos4.pose.position.y=0
    pos4.pose.position.z=3

    setpoints.append(pos4)

    pos5=PoseStamped()
    pos5.pose.position.x=3
    pos5.pose.position.y=3
    pos5.pose.position.z=3

    setpoints.append(pos5)

    pos6=PoseStamped()
    pos6.pose.position.x=3
    pos6.pose.position.y=3
    pos6.pose.position.z=-0.08737482875585556

    setpoints.append(pos6)

    pos7=PoseStamped()
    pos7.pose.position.x=3
    pos7.pose.position.y=3
    pos7.pose.position.z=3

    setpoints.append(pos7)

    pos8=PoseStamped()
    pos8.pose.position.x=0
    pos8.pose.position.y=0
    pos8.pose.position.z=3

    setpoints.append(pos8)

    pos9=PoseStamped()
    pos9.pose.position.x=0
    pos9.pose.position.y=0
    pos9.pose.position.z=0

    # setpoints.append(pos9)



    rospy.Subscriber("/mavros/state",State,stateMt.stateCb)
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped,psMt.psCb)
    rospy.Subscriber('/gripper_check',String,gripperMt.gripperCb)

    for i in range(100):
        local_pos_pub.publish(pos1)
        rate.sleep()

    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!")

    ofb_ctl.offboard_set_mode()
    print("OFFBOARD mode activated!")

    i=0
    while not rospy.is_shutdown():
        while( not psMt.is_reached(setpoints[i],0.05) and (i <=len(setpoints))):
            rospy.loginfo("Inside while not psMt.is_reached; Publishing setpoint {}".format(i))
            local_pos_pub.publish(setpoints[i])
            rate.sleep()

        rospy.loginfo("Published setpoint {}".format(i))

        if(i==2):
            rospy.loginfo('Inside i==2')
            if(gripperMt.status=="True"):
                rospy.loginfo('Ready to activate gripper')
                sfsrv=False
                while(not sfsrv):
                    sfsrv=gripperMt.gripper_action(True)
                    rospy.loginfo('srsrv:{}'.format(sfsrv))
            else:
                continue
        if(i==5):
            # sfsrv=True
            rospy.loginfo('Ready to deactivate gripper')
            gripperMt.gripper_action(False)

        i=i+1

        if i>=(len(setpoints)):
            rospy.loginfo("AUTO LAND")
            ofb_ctl.autoland_set_mode()
            break




if __name__=='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
