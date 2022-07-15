#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from std_msgs.msg import String
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
import cv2
import numpy as np
# import ArUco_library as aruco_library
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError


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
class image_proc():
    def __init__(self):
        self.image_sub=rospy.Subscriber('/eDrone/camera/image_raw',Image,self.image_callback)
        self.img=np.empty([])
        self.bridge=CvBridge()
        self.aruco_position=PoseStamped()
    def detect_ArUco(self,img):
        Detected_ArUco_markers = {}
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250 )
        parameters=aruco.DetectorParameters_create()
        corners,ids,_=aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        if ids is None:
            return 1
        for i in range(len(ids)):
            Detected_ArUco_markers[ids[i][0]]=corners[i]
        return Detected_ArUco_markers

    def image_callback(self,data):
        self.img=self.bridge.imgmsg_to_cv2(data,'bgr8')
        mask=np.zeros((400,400),dtype='uint8')
        # cv2.rectangle(mask,(185,190),(215,230),255,-1)
        cv2.rectangle(mask,(185,0),(215,400),255,-1)
        self.img=cv2.bitwise_and(self.img,self.img,mask=mask)
        temp=self.detect_ArUco(self.img)
        if temp!=1:
            self.aruco_position=psMt.ps
            rospy.loginfo("Aruco marker is at {}".format(self.aruco_position))
            
            # ofb_ctl.autoland_set_mode()
            # for i in range(10):
                # print(1)
            # return
        
#    res = cv2.flip(res, 0)
    
       
    def img_show(self):
        cv2.imshow("Drone image stream",self.img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit
        
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
            rospy.loginfo('status_fromService.result: {}'.format(status_fromService.result))
            return status_fromService
        except rospy.ServiceException as e:
            print('Service gripper action call failed: %s',e)

    def gripper_status(self):
        pass


ofb_ctl=offboard_control()
psMt=psMoniter()
def main():
    img_aruco=image_proc()
    bridge=CvBridge()
    # ofb_ctl=offboard_control()
    stateMt=stateMoniter()
    # psMt=psMoniter()
    gripperMt=gripperMoniter()

    local_pos_pub=rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)
    local_vel_pub=rospy.Publisher('mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

    rate=rospy.Rate(30.0)

    setpoints=[]
    vel_points=[]

    

    pos1=PoseStamped()
    pos1.pose.position.x=0
    pos1.pose.position.y=0
    pos1.pose.position.z=3

    vel1=TwistStamped()
    vel1.twist.linear.x=2
    vel1.twist.linear.y=0
    vel1.twist.linear.z=0

    vel_points.append(vel1)
    setpoints.append(pos1)

    pos2=PoseStamped()
    pos2.pose.position.x=9
    pos2.pose.position.y=0
    pos2.pose.position.z=3

    

    
    setpoints.append(pos2)

    pos3=PoseStamped()
    pos3.pose.position.x=9
    pos3.pose.position.y=0
    pos3.pose.position.z=3

     

    setpoints.append(pos3)

    pos4=PoseStamped()
    pos4.pose.position.x=9
    pos4.pose.position.y=0
    pos4.pose.position.z=-0.4

     

    setpoints.append(pos4)

    pos5=PoseStamped()
    pos5.pose.position.x=9
    pos5.pose.position.y=0
    pos5.pose.position.z=3

     

    setpoints.append(pos5)

    pos6=PoseStamped()
    pos6.pose.position.x=0
    pos6.pose.position.y=0
    pos6.pose.position.z=3

     

    setpoints.append(pos6)

    pos7=PoseStamped()
    pos7.pose.position.x=0
    pos7.pose.position.y=0
    pos7.pose.position.z=0

     
 


    



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
    k=0
    flag=1
    togle=1
    while not rospy.is_shutdown():
        while( not psMt.is_reached(setpoints[i],0.05) and (i <=len(setpoints))):
            rospy.loginfo("Publishing setpoint {}".format(i))
            if i==1 and flag==1:
                if togle:
                    vel1.header.stamp=rospy.Time.now()
                    local_vel_pub.publish(vel1)
                    togle=0
                else:
                    togle=1
                    
                    psMt.ps.pose.position.z=3
                    local_pos_pub.publish(psMt.ps)
            
            else:
                local_pos_pub.publish(setpoints[i])
            if (img_aruco.aruco_position.pose.position.x!=0) and (img_aruco.aruco_position.pose.position.y!=0) and (img_aruco.aruco_position.pose.position.z!=0):
                setpoints[1].pose.position.x=img_aruco.aruco_position.pose.position.x
                flag=0
            img_aruco.img_show()
             
            rate.sleep()
        
        rospy.loginfo("Published setpoint {}".format(i))
       
       
        if k==1:
            
            setpoints[1].pose.position.z=-0.4
            k+=1
            continue
        elif k==2:
            rospy.loginfo('Inside i==2')
            if(gripperMt.status=="True"):
                rospy.loginfo('Ready to activate gripper')
                sfsrv=False
                while(not sfsrv):
                    sfsrv=gripperMt.gripper_action(True)
                    rospy.logwarn_once('srsrv:{}'.format(sfsrv))
            else:
                rospy.loginfo('Gripper=false')
                continue
            
            setpoints[1].pose.position.z=3
            k+=1
            continue
            
        elif k==5:
            rospy.loginfo('Inside k==4')
            gripperMt.gripper_action(False)
            
        i=i+1
        k+=1
        if i>=(len(setpoints)):
            rospy.loginfo("AUTO LAND")
            ofb_ctl.autoland_set_mode()
            break


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
