#!/usr/bin/env python3
from multiprocessing import Process
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from std_msgs.msg import String
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco

class Drone:

    def __init__(self,drone_num):
        """Creates Drone Functionality"""
        self.drone_num=drone_num 
        self.drone_name='edrone'+str(drone_num)
        self.rate=0
        self.stateMt=self.stateMoniter(self)
        self.psMt=self.psMoniter(self)
        self.img_aruco=self.image_proc(self)
        self.gripperMt=self.gripperMoniter(self)
        self.ofb_ctl=self.offboard_control(self)

    def set_rate(self,rate):
        """Sets rate (in Hz)"""
        self.rate=rospy.Rate(rate)
        rospy.loginfo("Rate  has been set")

    class offboard_control:
        def __init__(self,outer):
            """Permits the usage of outer class objects"""

            self.outer=outer

            #publishers
            self.local_pos_pub=rospy.Publisher('/edrone'+str(self.outer.drone_num)+'/mavros/setpoint_position/local',PoseStamped,queue_size=10)
            self.local_vel_pub=rospy.Publisher('/edrone'+str(self.outer.drone_num)+'/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

            #subscribers
            rospy.Subscriber('/edrone'+str(self.outer.drone_num)+'/mavros/state',State,self.outer.stateMt.stateCb)
            rospy.Subscriber('/edrone'+str(self.outer.drone_num)+'/mavros/local_position/pose',PoseStamped,self.outer.psMt.psCb)
            rospy.Subscriber('/edrone'+str(self.outer.drone_num)+'/gripper_check',String,self.outer.gripperMt.gripperCb)

        def setArm(self):
            """Arms the drone"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming')
            try:
                armService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming',mavros_msgs.srv.CommandBool)
                armService(True)
            except rospy.ServiceException as e:
                print('Service arm call failed: %s',e)

        def setDisarm(self):
            """Disarms the drone"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming')
            try:
                armService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming',mavros_msgs.srv.CommandBool)
                armService(False)
            except rospy.ServiceException as e:
                print('Service disarm call failed: %s',e) 

        def offboard_set_mode(self):
            """Sets the drone to OFFBOARD mode"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/set_mode')
            try:
                flightModeService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/set_mode',mavros_msgs.srv.SetMode)
                flightModeService(custom_mode='OFFBOARD')
            except rospy.ServiceException as e:
                print('Service set_mode call failed: %s. Offboard mode could not be set.',e)

        def autoland_set_mode(self):
            """Performs autolanding and disarming"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/set_mode')
            try:
                flightModeService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/set_mode',mavros_msgs.srv.SetMode)
                flightModeService(custom_mode='AUTO.LAND')
            except rospy.ServiceException as e:
                print('Service autoland call failed: %s',e)

    class stateMoniter:
        """Performs State based operations"""
        def __init__(self,outer):
            """Initializes a State object"""
            self.outer=outer
            self.state=State()
        
        def stateCb(self,msg):
            """Callback function for the rostopic /mavros/state"""
            self.state=msg

    class psMoniter:
        """Performs Pose Stamped based operations"""
        def __init__(self,outer):
            """Initializes a PoseStamped object"""
            self.outer=outer
            self.ps=PoseStamped()
            self.pos=PoseStamped()
            self.vel=TwistStamped()
            self.vel_flag=False
    

            self.vel.twist.linear.x=2
            self.vel.twist.linear.y=0
            self.vel.twist.linear.z=0

            self.del_x=0
            self.del_y=0
            self.del_z=0

        def psCb(self,msg):
            """Callback functio for the rostopic /mavros/local_position/pose"""
            self.ps=msg
             

        def is_reached(self,other,thresh,extra_precision_z=0):
            """Checks whether the drone has reached the currently publishing setpoint"""
            self.del_x=abs(other.pose.position.x - self.ps.pose.position.x)
            self.del_y=abs(other.pose.position.y - self.ps.pose.position.y)
            self.del_z=abs(other.pose.position.z - self.ps.pose.position.z)
            return (self.del_x<=thresh and self.del_y<=thresh and self.del_z<=thresh-extra_precision_z)
            
        def goto(self,pos,extra_precision_z,vel_flag=False):
            self.pos.pose.position.x=pos[0]
            self.pos.pose.position.y=pos[1]
            self.pos.pose.position.z=pos[2]
            self.vel_flag=vel_flag
            self.extra_precision_z=extra_precision_z
            while (not self.is_reached(self.pos,0.2,self.extra_precision_z)):
                if self.vel_flag:
                    self.vel.header.stamp=rospy.Time.now()
                    self.outer.ofb_ctl.local_vel_pub.publish(self.vel)
                else:
                    self.outer.ofb_ctl.local_pos_pub.publish(self.pos)

                # If aruco is detected, setpoint is updated  
                if (self.outer.img_aruco.aruco_position.pose.position.x!=0) and (self.outer.img_aruco.aruco_position.pose.position.y!=0) and (self.outer.img_aruco.aruco_position.pose.position.z!=0) and (self.vel_flag):
                    rospy.loginfo("Aruco detected")
                    self.pos.pose.position.x= self.outer.img_aruco.aruco_position.pose.position.x
                    self.vel_flag=False
        
                self.outer.img_aruco.img_show()
                self.outer.rate.sleep()
            rospy.loginfo("Published setpoint {}".format(tuple(pos)))
            pos[0]=self.pos.pose.position.x
            pos[1]=self.pos.pose.position.y
            pos[2]=self.pos.pose.position.z
            
            return pos
        

    class image_proc():
        """Performs Image processing related operations"""
        def __init__(self,outer):
            """Initializes the objects necessary for image processing"""
            self.outer=outer
            self.image_sub=rospy.Subscriber('edrone'+str(self.outer.drone_num)+'/camera/image_raw',Image,self.image_callback)
            self.img=np.empty([])
            self.bridge=CvBridge()
            self.aruco_position=PoseStamped()

        def detect_ArUco(self,img):
            """Performs ArUco marker detection"""
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
            """Callback function for the rostopic /mavros/camera/image_raw 
               Peforms bitwise Mask (horizontally centered wrt to the drone's frame of reference) 
            """

            self.img=self.bridge.imgmsg_to_cv2(data,'bgr8')
            mask=np.zeros((400,400),dtype='uint8')
            #cv2.rectangle(mask,(185,190),(215,230),255,-1)
            cv2.rectangle(mask,(185,0),(215,400),255,-1)
            self.img=cv2.bitwise_and(self.img,self.img,mask=mask)
            Detected_ArUco_markers=self.detect_ArUco(self.img)

            if  Detected_ArUco_markers!=1:
                self.aruco_position=self.outer.psMt.ps
                rospy.loginfo("Aruco marker is at {}".format(self.aruco_position))
                
        def img_show(self):
            """Displays the drone's camera stream"""

            cv2.imshow("Drone image stream",self.img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return
            
    class gripperMoniter:
        """Performs operations related to the rosservice /active_gripper"""
        def __init__(self,outer):
            """Initializes the gripper class"""
            self.outer=outer
            self.status="False"

        def gripperCb(self,msg):
            """Callback function for the rostopic /edrone/gripper_check"""
            self.status=msg.data

        def gripper_action(self,action):
            """Activates or Deactivates the drone's gripper"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/activate_gripper')
            try:
                gripperActionService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/activate_gripper',Gripper)
                status_fromService=gripperActionService(activate_gripper=action)
                rospy.loginfo('Status[Service]: {}'.format(status_fromService.result))
                return status_fromService.result
            except rospy.ServiceException as e:
                print('Service gripper action call failed: %s',e)

        def gripper_status(self):
            pass
def edrone0_main(edrone0):

    """
    * Function Name: edrone0_main
    * Input: edrone0,shared_dict
    * Output: None
    * Logic: Instructions for the edrone0
    * Example Call: edrone0_main(edrone0,shared_dict)
    """
     
    rospy.init_node('multi_drone_1')
    edrone0.set_rate(30)
    
    #      above home,start-c1,end-c1,  above blue-truck,on blue-truck,above blue-truck,above home,home
    edrone0_sp=[[0,0,3],[0,16,3],[1,16,3],[62,16,3],[15.7,-8.4,4],[15.7,-8.4,1.9],[15.7,-8.4,4],[0,0,4],[0,0,0]]
    
    dummy_sp=PoseStamped()
    for _ in range(100):
        edrone0.ofb_ctl.local_pos_pub.publish(dummy_sp) 

    # Arming the vehicle
    while not edrone0.stateMt.state.armed:
        edrone0.ofb_ctl.setArm()
        edrone0.rate.sleep()
    rospy.loginfo('['+edrone0.drone_name+']'+'!ARMED')

    # Activate off-board mode
    edrone0.ofb_ctl.offboard_set_mode()
    rospy.loginfo('['+edrone0.drone_name+']:'+'OFFBOARD mode activated!')

    # initialing variables
    i=0
    setpoint_finished=0
    vel_flag=1
    no_of_box=2
    detached_boxes=0

    while not rospy.is_shutdown():


        if i==3 and vel_flag==1:
            rospy.loginfo('['+edrone0.drone_name+']'+'velocity command')
            edrone0_sp[3]=edrone0.psMt.goto(edrone0_sp[i],0,True)
        else:
            edrone0.psMt.goto(edrone0_sp[i],0)

        if setpoint_finished==3:
            # To get the drone on aruco
            print(edrone0_sp[2])
            edrone0_sp[3][2]=0
            vel_flag=0
            edrone0.psMt.goto(edrone0_sp[i],0.05)
            setpoint_finished+=1
            continue

        elif setpoint_finished==4:
            rospy.loginfo('['+edrone0.drone_name+']'+'Inside i==4')
            if(edrone0.gripperMt.status=="True"):
                rospy.loginfo('['+edrone0.drone_name+']'+'Ready to activate gripper')
                sfsrv=False
                while(not sfsrv):
                    sfsrv=edrone0.gripperMt.gripper_action(True)
                    rospy.logwarn_once('srsrv:{}'.format(sfsrv))
            else:
                rospy.loginfo('['+edrone0.drone_name+']'+'Gripper=false')
                continue
            edrone0_sp[3][2]=3
            setpoint_finished+=1
            continue

        elif setpoint_finished==7:
            rospy.loginfo('['+edrone0.drone_name+']'+'Inside setpoint_finished==6')
            gripper_result=True
            while(gripper_result):
                gripper_result=edrone0.gripperMt.gripper_action(False)
            detached_boxes+=1

        if i==6 and detached_boxes<no_of_box:
            i=0
            setpoint_finished=0
            vel_flag=1
            edrone0_sp[1][1]+=8
            edrone0_sp[1][2]=4
            edrone0_sp[2][1]+=8
            edrone0_sp[3][1]+=8
            edrone0_sp[5][2]=2
            edrone0_sp[3][0]=62
            edrone0.img_aruco.aruco_position.pose.position.x=0
            edrone0.img_aruco.aruco_position.pose.position.y=0
            edrone0.img_aruco.aruco_position.pose.position.z=0
        i=i+1
        setpoint_finished+=1
        if i>=(len(edrone0_sp)):
            rospy.loginfo('['+edrone0.drone_name+']'+'AUTO LAND')
            edrone0.ofb_ctl.autoland_set_mode()
            break

def edrone1_main(edrone1):
    """
    * Function Name: edrone1_main
    * Input: edrone1,shared_dict
    * Output: None
    * Logic: Instructions for the edrone1
    * Example Call: edrone1_main(edrone1,shared_dict)
    """

    rospy.init_node('multi_drone_2')

    edrone1.set_rate(30)
    
    #      above home,start-c1,end-c1,  above red-truck,on red-truck,above red-truck,above home,home
    edrone1_sp=[[0,0,3],[0,-12,3],[1,-12,3],[62,-12,3],[60,4,4],[60,4,1.9],[60,4,4],[0,0,4],[0,0,0]]
    
    dummy_sp=PoseStamped()
    for _ in range(100):
        edrone1.ofb_ctl.local_pos_pub.publish(dummy_sp) 

    # Arming the vehicle
    while not edrone1.stateMt.state.armed:
        edrone1.ofb_ctl.setArm()
        edrone1.rate.sleep()
    rospy.loginfo('['+edrone1.drone_name+']'+'Armed!')

    # Activate off-board mode
    edrone1.ofb_ctl.offboard_set_mode()
    rospy.loginfo('['+edrone1.drone_name+']'+'OFFBOARD mode activated!')

    # initialing variables
    i=0
    setpoint_finished=0
    vel_flag=1
    no_of_box=2
    detached_boxes=0

    while not rospy.is_shutdown():


        if i==3 and vel_flag==1:
            rospy.loginfo('['+edrone1.drone_name+']'+'velocity command')
            edrone1_sp[3]=edrone1.psMt.goto(edrone1_sp[i],0,True)
        else:
            edrone1.psMt.goto(edrone1_sp[i],0)

        if setpoint_finished==3:
            # To get the drone on aruco
            print(edrone1_sp[2])
            edrone1_sp[3][2]=0
            vel_flag=0
            setpoint_finished+=1
            edrone1.psMt.goto(edrone1_sp[i],0.05)
            continue

        elif setpoint_finished==4:
            rospy.loginfo('['+edrone1.drone_name+']'+'Inside i==4')
            if(edrone1.gripperMt.status=="True"):
                rospy.loginfo('Ready to activate gripper')
                sfsrv=False
                while(not sfsrv):
                    sfsrv=edrone1.gripperMt.gripper_action(True)
                    rospy.logwarn_once('srsrv:{}'.format(sfsrv))
            else:
                rospy.loginfo('['+edrone1.drone_name+']'+'Gripper=false')
                continue
            edrone1_sp[3][2]=3
            setpoint_finished+=1
            continue

        elif setpoint_finished==7:
            rospy.loginfo('['+edrone1.drone_name+']'+'Inside setpoint_finished==6')
            gripper_result=True
            while(gripper_result):
                gripper_result=edrone1.gripperMt.gripper_action(False)
            detached_boxes+=1

        if i==6 and detached_boxes<no_of_box:
            i=0
            setpoint_finished=0
            vel_flag=1
            edrone1_sp[1][2]=4
            edrone1_sp[1][1]-=20
            edrone1_sp[2][1]-=20
            edrone1_sp[3][1]-=20
            edrone1_sp[5][2]=1.904
            edrone1_sp[3][0]=62
            edrone1.img_aruco.aruco_position.pose.position.x=0
            edrone1.img_aruco.aruco_position.pose.position.y=0
            edrone1.img_aruco.aruco_position.pose.position.z=0
        i=i+1
        setpoint_finished+=1
        if i>=(len(edrone1_sp)):
            rospy.loginfo('['+edrone1.drone_name+']'+'AUTO LAND')
            edrone1.ofb_ctl.autoland_set_mode()
            break
def main():

    #Creating Drone instances
    edrone0=Drone(0)
    edrone1=Drone(1)
 
    #Assigning different cores for different drones (Multiprocessing)
    process0=Process(target=edrone0_main,args=(edrone0,))
    process1=Process(target=edrone1_main,args=(edrone1,))

    #Starting the processes 
    process0.start()
    process1.start()
     
    #Joining the processes
    process0.join()
    process1.join()        

if __name__=='__main__':
    """Main function"""
    try:
        main()
    except rospy.ROSInitException as e:
       rospy.logerr_once(e)
