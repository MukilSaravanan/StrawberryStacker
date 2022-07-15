#!/usr/bin/env python3
from multiprocessing import Process,Manager
from pydoc import splitdoc
from time import sleep
from turtle import shearfactor
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from std_msgs.msg import String, UInt8
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco


"""
* Team Id : 1929
* Author List : Mukil Saravanan, Sanjay Kumar M, Hariraj A, Anitha K
* Filename: SS_1929_strawberry_stacker.py
* Theme: Strawberry Stacker
* Functions: edrone0_main,edrone1_main
* Global Variables: none
"""

class Drone:
    """Creates Drone Functionality"""
    def __init__(self,drone_num):
        self.drone_num=drone_num 
        self.drone_name='edrone'+str(drone_num)
        self.stateMt=self.stateMoniter(self)
        self.psMt=self.psMoniter(self)
        self.img_aruco=self.image_proc(self)
        self.gripperMt=self.gripperMoniter(self)
        self.ofb_ctl=self.offboard_control(self)
        self.rand_spwn=self.random_spawn(self)
        self.rate=0
        self.vel_rate=0



    def set_rate(self,rate,vel_rate=50):
        """Sets rate (in Hz)"""
        self.rate=rospy.Rate(rate)
        self.vel_rate=rospy.Rate(vel_rate)
        rospy.loginfo('['+self.drone_name+']: '+"Rate  has been set")

    class offboard_control:
        """Performs """

        def __init__(self,outer):
            """Permits the usage of outer class objects"""
            self.outer=outer

        def setArm(self):
            """Arms the drone"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming')
            try:
                armService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming',mavros_msgs.srv.CommandBool)
                armService(True)
            except rospy.ServiceException as e:
                rospy.logerr_once('['+self.outer.drone_name+']: '+'Service arm call failed: %s',e)

        def setDisarm(self):
            """Disarms the drone"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming')
            try:
                armService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/cmd/arming',mavros_msgs.srv.CommandBool)
                armService(False)
            except rospy.ServiceException as e:
                rospy.logerr_once('['+self.outer.drone_name+']: '+'Service disarm call failed: %s',e) 

        def offboard_set_mode(self):
            """Sets the drone to OFFBOARD mode"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/set_mode')
            try:
                flightModeService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/set_mode',mavros_msgs.srv.SetMode)
                flightModeService(custom_mode="OFFBOARD")
            except rospy.ServiceException as e:
                rospy.logerr_once('['+self.outer.drone_name+']: ' +'Service set_mode call failed: %s. Offboard mode could not be set.',e)

        def autoland_set_mode(self):
            """Performs autolanding and disarming"""
            rospy.wait_for_service('edrone'+str(self.outer.drone_num)+'/mavros/set_mode')
            try:
                flightModeService=rospy.ServiceProxy('edrone'+str(self.outer.drone_num)+'/mavros/set_mode',mavros_msgs.srv.SetMode)
                flightModeService(custom_mode='AUTO.LAND')
            except rospy.ServiceException as e:
                rospy.logerr_once('['+self.outer.drone_num+']: '+'Service autoland call failed: %s',e)

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
            '''Initializes error values in the x,y,z axis'''
            self.del_x=0
            self.del_y=0
            self.del_z=0

        def psCb(self,msg):
            """Callback functio for the rostopic /mavros/local_position/pose"""

            self.ps=msg
             
        def is_reached(self,other,thresh,extra_precision=0):
            """Checks whether the drone has reached the currently publishing setpoint"""

            self.del_x=abs(other.pose.position.x - self.ps.pose.position.x)
            self.del_y=abs(other.pose.position.y - self.ps.pose.position.y)
            self.del_z=abs(other.pose.position.z - self.ps.pose.position.z)
            # self.del_orientation_y=abs(other.pose.orientation.y - self.ps.pose.orientation.y)
            return (self.del_x<=thresh-extra_precision and self.del_y<=thresh-extra_precision and self.del_z<=thresh)
            
        

    class image_proc():
        """Performs Image processing related operations"""
        
        def __init__(self,outer):
            """Initializes the objects necessary for image processing"""
            self.outer=outer
            self.img_sub=None
            self.img=np.empty([])
            self.bridge=CvBridge() # Bridge betweeen Opencv and ROS
            self.aruco_position=PoseStamped()
            self.temp_aruco_id=0
            self.aruco_id=0
            self.k=0

        def detect_ArUco(self,img):
            """Performs ArUco marker detection"""

            gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #converts colormap
            aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250 )
            parameters=aruco.DetectorParameters_create()
            corners,self.temp_aruco_id,_=aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
            # return self.temp_aruco_id

        def image_callback(self,data):
            """Callback function for the rostopic /mavros/camera/image_raw 
               Peforms bitwise Mask (horizontally centered wrt to the drone's frame of reference) 
            """

            self.img=self.bridge.imgmsg_to_cv2(data,'bgr8')
            mask=np.zeros((400,400),dtype='uint8')
            #cv2.rectangle(mask,(185,190),(215,230),255,-1)
            cv2.rectangle(mask,(185,0),(215,400),255,-1)
            self.img=cv2.bitwise_and(self.img,self.img,mask=mask) 
            self.detect_ArUco(self.img)

        def update_aruco_position(self):
            """Stores the local position of the drone when ArUco marker is detected"""

            if self.temp_aruco_id is not None:
                self.k+=1
                self.aruco_position=self.outer.psMt.ps
                try:
                    self.aruco_id=self.temp_aruco_id[0][0]
                except TypeError: #Ignores the rare case sudden change to None after coming inside the if block
                    pass
                # rospy.loginfo('['+self.outer.drone_name+']: '+"Aruco marker is at {} with id {}".format(self.aruco_position.pose.position,self.aruco_id))
                
            else:
                if not (self.k ==0):
                    print("detect_accuracy",self.k) #prints the detection accuracy of ArUco marker
                    self.k=0

        def img_show(self):
            """Displays the drone's camera stream"""


            cv2.imshow("[edrone"+self.outer.drone_name+" Image Stream"+']',self.img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit
            
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
                rospy.loginfo('['+self.outer.drone_name+']: '+'Status[Service]: {}'.format(status_fromService.result))
                return status_fromService.result
            except rospy.ServiceException as e:
                rospy.logerr_once('['+self.outer.drone_name+']: '+'Service gripper action call failed: %s',e)

        def gripper_status(self):
            pass
    
    class random_spawn:
        """Class for managing incoming spawn messages"""

        def __init__(self,outer):
            """Initializes the total box count and nth_row"""
            self.outer=outer
            self.box_count=0
            self.nth_row=0
            # self.row_data_dict={-1:[self.box_count,0,0]}

 
def edrone0_main(edrone0,shared_dict):

    """
    * Function Name: edrone0_main
    * Input: edrone0,shared_dict
    * Output: None
    * Logic: Instructions for the edrone0
    * Example Call: edrone0_main(edrone0,shared_dict)
    """

    rospy.init_node('multi_drone_1')
    edrone0.set_rate(30)

    #Publishers
    local_pos_pub=rospy.Publisher('/edrone'+str(edrone0.drone_num)+'/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    local_vel_pub=rospy.Publisher('/edrone'+str(edrone0.drone_num)+'/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

    #subscribers
    rospy.Subscriber('/edrone'+str(edrone0.drone_num)+'/mavros/state',State,edrone0.stateMt.stateCb)
    rospy.Subscriber('/edrone'+str(edrone0.drone_num)+'/mavros/local_position/pose',PoseStamped,edrone0.psMt.psCb)
    rospy.Subscriber('/edrone'+str(edrone0.drone_num)+'/gripper_check',String,edrone0.gripperMt.gripperCb)
    edrone0.img_aruco.image_sub=rospy.Subscriber('edrone'+str(edrone0.drone_num)+'/camera/image_raw',Image,edrone0.img_aruco.image_callback)

    #Initializes the local variables
    dest_truck_x=0
    dest_truck_y=0
    cell_width=1.23
    cell_length=0.85

    shared_dict[-1]=manager.list()
    shared_dict[-1].append(0)
    shared_dict[-1].append(0)
    shared_dict[-1].append(0)
 
    def dummy(msg):

        edrone0.rand_spwn.box_count+=1
        
        

        if msg.data not in shared_dict :
            shared_dict[msg.data]=manager.list()
            shared_dict[msg.data].append(0)
            shared_dict[msg.data].append(0)
            shared_dict[msg.data].append(0)
        shared_dict[msg.data][0]+=1   #dict_name={-1:[total_count,0,0],row_num:[box_count,lastly_found,working status]}
        shared_dict[-1][0]+=1
        print("total count",shared_dict[-1][0])
        print("New row added:",msg.data)
        # rospy.loginfo_once("spawn_row_msg:",shared_dict)

    def spawnCb(msg):
        dummy(msg)
        
    rospy.Subscriber('/spawn_info',UInt8,spawnCb)


    #      above home,start-c1,incremental-value,end-c1,above blue-truck,on blue-truck,above blue-truck,above home,home
    edrone0_sp=[[0,0,3],[-3,edrone0.rand_spwn.nth_row,6],[-1,edrone0.rand_spwn.nth_row,3],[50,edrone0.rand_spwn.nth_row,3],[dest_truck_x,dest_truck_y,6],[dest_truck_x,dest_truck_y,1.9],[dest_truck_x,dest_truck_y,6],[0,0,6],[0,0,0]]        

    # waiting for FCU connection
    while(not edrone0.stateMt.state.connected):
        edrone0.rate.sleep()

    # Arming the vehicle
    while not edrone0.stateMt.state.armed:
        edrone0.ofb_ctl.setArm()
        edrone0.rate.sleep()
    rospy.loginfo('['+edrone0.drone_name+']: '+'!ARMED')
    
    dummy_sp=PoseStamped()
    #Changing to OFFBOARD mode
    while not edrone0.stateMt.state.mode == 'OFFBOARD':
        local_pos_pub.publish(dummy_sp)
        edrone0.ofb_ctl.offboard_set_mode()
        edrone0.rate.sleep()
    rospy.loginfo('['+edrone0.drone_name+']: '+'OFFBOARD mode activated!')


    # initialing variables
    i=0
    setpoint_finished=0
    vel_flag=1
    detached_boxes_on_two_trucks=0
    detached_boxes_on_one_truck=0
    detached_box_blue=0
    detached_box_red=0
    waiting_list_flag=0
    destination_set_flag=0
    gripper_failed=0

    pos=PoseStamped()
    z_correction=PoseStamped()
    vel=TwistStamped()

    vel.twist.linear.x=2
    vel.twist.linear.y=0
    vel.twist.linear.z=0

    extra_precision_z=0
    z_correcting_flag=1
    history=-10
    exception_flag=1
    waiting_list_boxes=[] #[x_position,nth_row,aruco_id]


    # edrone0.rand_spwn.row_data_dict={-8:[2,0,0],9:[2,0,0]}

    blue_truck_wrt_edrone0=(16.55,-8.4)
    # blue_truck_wrt_edrone0=(15.7,-8.4)
    # red_truck_wrt_edrone0=(59.2,63.75)
    red_truck_wrt_edrone0=(57.9,63.75)
    m=-1 #multiplier for cell width
    n=-1 #multiplier for cell length
    token_num=0


    last_sp=[-1,-1,-1]

    # shared_dict={1:[1,0,0],2:[1,0,0],3:[1,0,0],4:[1,0,0],5:[1,0,0],6:[1,0,0],7:[1,0,0],8:[1,0,0],-1:[8,0,0]}

    while not rospy.is_shutdown():
        
        if i==1:
            if shared_dict[-1][0]<1:#fail safe when no boxes 
                local_pos_pub.publish(dummy_sp)
                edrone0.rate.sleep()
                continue
            # find minimum flag it
            row_data_dict_keys=list(shared_dict.keys())
            edrone0.rand_spwn.nth_row=min(row_data_dict_keys)
            while( (not shared_dict[edrone0.rand_spwn.nth_row][2])==0) or shared_dict[edrone0.rand_spwn.nth_row][0]==0 or edrone0.rand_spwn.nth_row==-1:
                try:
                    # rospy.loginfo(edrone0.rand_spwn.nth_row)
                    # rospy.loginfo(row_data_dict_keys) 
                    row_data_dict_keys.remove(edrone0.rand_spwn.nth_row)
                    # rospy.loginfo(row_data_dict_keys) 
                    edrone0.rand_spwn.nth_row=min(row_data_dict_keys)
                    # rospy.loginfo(edrone0.rand_spwn.nth_row)

                except:
                    rospy.logwarn("[edrone"+str(edrone0.drone_num)+"]: exception caught")
                    exception_flag=0
                    i=6
                    waiting_list_flag=1
                    break
           
            print("[EDRONE0]:KEYS:{}".format(row_data_dict_keys))
            # manipulate_dict(edrone0.rand_spwn.nth_row,2) #setting working flag status
            shared_dict[edrone0.rand_spwn.nth_row][2]=1 #setting working flag status
            # print(edrone0.rand_spwn.nth_row)
            
            nth_row_distance=(edrone0.rand_spwn.nth_row-1)*4 # setting nth row to go
            edrone0_sp[1][1]=nth_row_distance
            edrone0_sp[2][1]=nth_row_distance
            edrone0_sp[3][1]=nth_row_distance

            edrone0_sp[1][0]=(shared_dict[edrone0.rand_spwn.nth_row][1])-3 #intermidiate x in edrone0.rand_spwn.nth_row
            edrone0_sp[2][0]=(shared_dict[edrone0.rand_spwn.nth_row][1])-1    #updating lastly found x in edrone0.rand_spwn.nth_row

            # print("shared_dict",shared_dict)
            # index+=2 # even index
            print("[edrone"+str(edrone0.drone_num) + "]: working on row ",edrone0.rand_spwn.nth_row)
     
        pos.pose.position.x=edrone0_sp[i][0]
        pos.pose.position.y=edrone0_sp[i][1]
        pos.pose.position.z=edrone0_sp[i][2]

        if exception_flag==1:

            while (not edrone0.psMt.is_reached(pos,0.2,extra_precision_z)):
                if vel_flag and i==3:
                    # Velocity command
                    vel.header.stamp=rospy.Time.now()
                    local_vel_pub.publish(vel)

                    # Failsafe mechanism when it reaches the end of a row
                    if edrone0.psMt.ps.pose.position.x>50:
                        i=6
                        setpoint_finished=0
                        shared_dict[edrone0.rand_spwn.nth_row][1]=0 #setting last seen  to zero because to find missed one
                        for i in waiting_list_boxes:
                            if edrone0.rand_spwn.nth_row==i[1]:
                                waiting_list_boxes.remove(i)
                        continue

                    # Corrects z position
                    elif edrone0.psMt.ps.pose.position.x>25 and z_correcting_flag==1:
                        z_correcting_flag=0
                        rospy.logwarn("correcting z")
                        z_correction=edrone0.psMt.ps
                        z_correction.pose.position.z=3
                        for _ in range(45):
                            local_pos_pub.publish(z_correction)
                            edrone0.rate.sleep()
                    
                    edrone0.img_aruco.update_aruco_position()

                    #Strategy (for minimizing the time of delivery and flying cost)
                    if (edrone0.img_aruco.aruco_position.pose.position.x!=0) and (edrone0.img_aruco.aruco_position.pose.position.y!=0) and (edrone0.img_aruco.aruco_position.pose.position.z!=0) and ( edrone0.img_aruco.aruco_position.pose.position.x-history>0.7):
                        rospy.logwarn("old History: {}".format(history))
                        rospy.loginfo('['+edrone0.drone_name+']: '+"Aruco detected")
                        history= edrone0.img_aruco.aruco_position.pose.position.x
                        rospy.logwarn("new history:{}".format(history))

                        # Appends lesser priority boxes to the waiting list
                        if (edrone0.img_aruco.aruco_id==1) and (not (edrone0.rand_spwn.nth_row>6 or shared_dict[edrone0.rand_spwn.nth_row][0]==1)):

                            rospy.loginfo('red box appended')
                            
                            waiting_list_boxes.append([edrone0.img_aruco.aruco_position.pose.position.x,edrone0.rand_spwn.nth_row,edrone0.img_aruco.aruco_id])

                            rospy.loginfo("waiting list :{}".format(waiting_list_boxes))

                        elif (edrone0.img_aruco.aruco_id==2) and (not (edrone0.rand_spwn.nth_row<=6 or shared_dict[edrone0.rand_spwn.nth_row][0]==1)):

                            rospy.loginfo('blue box appended')
                            waiting_list_boxes.append([edrone0.img_aruco.aruco_position.pose.position.x,edrone0.rand_spwn.nth_row,edrone0.img_aruco.aruco_id])
                            rospy.loginfo("waiting list:{}".format(waiting_list_boxes))

                        else:
                            # Sets higher priority
                            rospy.loginfo(' out of red or blue box')
                            pos.pose.position.x= edrone0.img_aruco.aruco_position.pose.position.x
                            vel_flag=False

                        if waiting_list_flag==0:
                            shared_dict[-1][0]-=1 #decreasing one box count
                            shared_dict[edrone0.rand_spwn.nth_row][0]-=1 #decreasing row box count 
                            shared_dict[edrone0.rand_spwn.nth_row][1]=edrone0.img_aruco.aruco_position.pose.position.x #upadating last aruco_box 

                        #Storing found aruco position
                        edrone0.img_aruco.aruco_position.pose.position.x=0
                        edrone0.img_aruco.aruco_position.pose.position.y=0
                        edrone0.img_aruco.aruco_position.pose.position.z=0
                        edrone0.vel_rate.sleep()

                else:
                    # print('inside goto pos cmd')
                    local_pos_pub.publish(pos)
                    edrone0.rate.sleep()

                # edrone0.img_aruco.img_show()
                
            edrone0_sp[i][0]=pos.pose.position.x
            edrone0_sp[i][1]=pos.pose.position.y
            edrone0_sp[i][2]=pos.pose.position.z
            
            # if edrone0_sp[i]!=last_sp:
            rospy.loginfo('['+ edrone0.drone_name+']: '+"Published setpoint {}".format(tuple(edrone0_sp[i])))
            rospy.loginfo("i:{}, spf:{}, vel_flag:{}".format(i,setpoint_finished,vel_flag))
                # last_sp=edrone0_sp[i]
            # else:
                # print("[edrone0]: last_sp:{} \n edrone_sp:{}".format(last_sp,edrone0_sp[i]))

        # if i==3 and vel_flag==1:
           
            # rospy.loginfo('['+edrone0.drone_name+']: '+'velocity command')
            # edrone0_sp[3]=edrone0.psMt.goto(edrone0_sp[i],0,True)

        if setpoint_finished==3:
            # To get the drone on aruco

            # print(edrone0_sp[3])
            edrone0_sp[3][2]=-0.1
            vel_flag=0
            extra_precision_z=0.1
            setpoint_finished+=1
            destination_set_flag=1
            continue

        elif setpoint_finished==4:

            
            # rospy.loginfo('['+edrone0.drone_name+']: '+'Inside i==4')
            if(edrone0.gripperMt.status=="True"):
                gripper_failed=0
                rospy.loginfo('['+edrone0.drone_name+']: '+'Ready to activate gripper')
                sfsrv=False
                while(not sfsrv):
                    sfsrv=edrone0.gripperMt.gripper_action(True)
                    rospy.loginfo_once('['+edrone0.drone_name+']: '+'srsrv:{}'.format(sfsrv))
            else:
                # rospy.logwarn_once('['+edrone0.drone_name+']: '+'Gripper=false')
                gripper_failed+=1
                if (gripper_failed==700):
                    rospy.logwarn_once("[edrone0]: Triggering gripper failed failsafe")
                    rospy.logwarn_once("edrone0_sp={}".format(edrone0_sp[2]))
                    i=2
                    setpoint_finished=2
                    vel_flag=1
                    history=-10
                    gripper_failed=0
                    shared_dict[edrone0.rand_spwn.nth_row][0]+=1
                continue
            
            #Setting destination truck
            if destination_set_flag:
                if edrone0.img_aruco.aruco_id == 1:  
                    dest_truck_base_x,dest_truck_base_y=red_truck_wrt_edrone0
                    detached_boxes_on_one_truck=detached_box_red
                    detached_box_red+=1
                
                else:   
                    dest_truck_base_x,dest_truck_base_y=blue_truck_wrt_edrone0
                    detached_boxes_on_one_truck=detached_box_blue
                    detached_box_blue+=1
                        
                # Stacking algorithm
                if detached_boxes_on_one_truck%12==0: 
                    #resets m,n when detached boxes are 12 (i.e.) when two rows are filled with 2 boxes
                    m=-1
                    n=-1

                if detached_boxes_on_one_truck%6==0:
                    m=-1 #resets m when last column is filled with 2 boxes
                    n+=1 #increamenting n after filling last column

                if detached_boxes_on_one_truck%2==0:
                    m+=1 #increamenting m after stacking two boxes in a cell
            
                dest_truck_x=dest_truck_base_x+(n*(cell_length/3)) # incrementing by cell width every two boxes
                dest_truck_y=dest_truck_base_y+(m*(cell_width)) #toggling at every three detached boxes

                rospy.logwarn_once("dest loc:({},{})".format(dest_truck_x,dest_truck_y))
                
                # assigning particular cell location to drop the box
                edrone0_sp[4][0:2]=dest_truck_x+0.85,dest_truck_y
                edrone0_sp[5][0:2]=dest_truck_x,dest_truck_y
                edrone0_sp[6][0:2]=dest_truck_x,dest_truck_y

                destination_set_flag=0

            

            edrone0_sp[3][2]=3
            setpoint_finished+=1
            extra_precision_z=0

            continue

        elif setpoint_finished==7:
            #Detaching box from the drone
            rospy.loginfo('['+edrone0.drone_name+']: '+'Inside setpoint_finished==6')
            gripper_result=True
            while(gripper_result):
                gripper_result=edrone0.gripperMt.gripper_action(False)
            detached_boxes_on_two_trucks+=1

        if i==6 and waiting_list_flag==0:
            # resetting variables so that it performs tasks for the next strawberry box
            i=0
            setpoint_finished=0
            vel_flag=1
            edrone0_sp[1][2]=6
            edrone0_sp[3]=[50,nth_row_distance,3]
            z_correcting_flag=1
            shared_dict[edrone0.rand_spwn.nth_row][2]=0 #resetting working flag status
            history=-10

        if i==6 and  waiting_list_flag==1:
            print("waiting list")
           
            if token_num<len(waiting_list_boxes):
                i=2
                setpoint_finished=2
                vel_flag=0
                exception_flag=1
                edrone0_sp[3]=[waiting_list_boxes[token_num][0],(waiting_list_boxes[token_num][1]-1)*4,6]
                edrone0.img_aruco.aruco_id=waiting_list_boxes[token_num][2]
                token_num+=1
                print('incrementing token num to {}'.format(token_num))

        i=i+1
        setpoint_finished+=1

        if i>=(len(edrone0_sp)):
            rospy.loginfo('['+edrone0.drone_name+']: '+'AUTO LAND')
            edrone0.ofb_ctl.autoland_set_mode()
            break

def edrone1_main(edrone1,shared_dict):

    """
    * Function Name: edrone1_main
    * Input: edrone1,shared_dict
    * Output: None
    * Logic: Instructions for the edrone1
    * Example Call: edrone1_main(edrone1,shared_dict)
    """

    rospy.init_node('multi_drone_2')
    
    edrone1.set_rate(30)

    #Publishers
    local_pos_pub=rospy.Publisher('/edrone'+str(edrone1.drone_num)+'/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    local_vel_pub=rospy.Publisher('/edrone'+str(edrone1.drone_num)+'/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

    #subscribers
    rospy.Subscriber('/edrone'+str(edrone1.drone_num)+'/mavros/state',State,edrone1.stateMt.stateCb)
    rospy.Subscriber('/edrone'+str(edrone1.drone_num)+'/mavros/local_position/pose',PoseStamped,edrone1.psMt.psCb)
    rospy.Subscriber('/edrone'+str(edrone1.drone_num)+'/gripper_check',String,edrone1.gripperMt.gripperCb)
    edrone1.img_aruco.image_sub=rospy.Subscriber('edrone'+str(edrone1.drone_num)+'/camera/image_raw',Image,edrone1.img_aruco.image_callback)

    #Initializes the local variables
    dest_truck_x=0
    dest_truck_y=0
    cell_width=1.23
    cell_length=0.85

    #      above home,start-c1,end-c1,  above red-truck,on red-truck,above red-truck,above home,home

    edrone1_sp=[[0,0,4],[-3,edrone1.rand_spwn.nth_row,5],[-1,edrone1.rand_spwn.nth_row,3],[50,edrone1.rand_spwn.nth_row,3],[dest_truck_x,dest_truck_y,5],[dest_truck_x,dest_truck_y,1.9],[dest_truck_x,dest_truck_y,5],[0,0,5],[0,0,0]]
    
        # waiting for FCU connection
    while(not edrone1.stateMt.state.connected):
        edrone1.rate.sleep()

    # Arming the vehicle
    while not edrone1.stateMt.state.armed:
        edrone1.ofb_ctl.setArm()
        edrone1.rate.sleep()
    rospy.loginfo('['+edrone1.drone_name+']: '+'Armed!')

    dummy_sp=PoseStamped()
    #Changing to OFFBOARD
    while not edrone1.stateMt.state.mode == 'OFFBOARD':
        local_pos_pub.publish(dummy_sp)
        edrone1.ofb_ctl.offboard_set_mode()
        edrone1.rate.sleep()
    rospy.loginfo('['+edrone1.drone_name+']: '+'OFFBOARD mode activated!')

    dummy_sp.pose.position.x=edrone1_sp[1][0]
    dummy_sp.pose.position.y=edrone1_sp[1][1]
    dummy_sp.pose.position.z=edrone1_sp[1][2]

    # initialing variables
    i=0
    setpoint_finished=0
    vel_flag=1
    detached_boxes_on_two_trucks=0
    detached_boxes_on_one_truck=0
    detached_box_blue=0
    detached_box_red=0
    gripper_failed=0

    pos=PoseStamped()
    z_correction=PoseStamped()

    vel=TwistStamped()

    vel.twist.linear.x=2
    vel.twist.linear.y=0
    vel.twist.linear.z=0

    extra_precision_z=0
    z_correcting_flag=1
    history=-10
    exception_flag=1
    waiting_list_boxes=[] #[x_position,nth_row,aruco_id]
    toggle=1
    waiting_list_flag=0
    destination_set_flag=0
    # shared_dict={2:[1,0,0],3:[1,0,0],4:[1,0,0],5:[1,0,0],6:[1,0,0],7:[1,0,0],8:[1,0,0],-1:[7,0,0]}

    last_sp=[-1,-1,-1]

    blue_truck_wrt_edrone1=(15+0.5,-68.4)
    red_truck_wrt_edrone1=(59.3,4)
    m=-1
    n=-1
    token_num=0
    while not rospy.is_shutdown():

        if i==1:
            if shared_dict[-1][0]<2:
                local_pos_pub.publish(dummy_sp)
                edrone1.rate.sleep()
                continue
            #find min 
            row_data_dict_keys=list(shared_dict.keys())
            # print('ROW DATA DICT KEYS:{}'.format(row_data_dict_keys))
            edrone1.rand_spwn.nth_row=max(row_data_dict_keys)
            while( not shared_dict[edrone1.rand_spwn.nth_row][2]==0 or shared_dict[edrone1.rand_spwn.nth_row][0]==0 or edrone1.rand_spwn.nth_row==-1):

                try:
                    # rospy.loginfo(edrone0.rand_spwn.nth_row)
                    # rospy.loginfo(row_data_dict_keys) 
                    row_data_dict_keys.remove(edrone1.rand_spwn.nth_row)
                    # rospy.loginfo(row_data_dict_keys) 
                    edrone1.rand_spwn.nth_row=max(row_data_dict_keys)
                    # rospy.loginfo(edrone0.rand_spwn.nth_row)
                except:
                    rospy.logwarn("[edrone"+str(edrone1.drone_num) +"]: exception caught")
                    exception_flag=0
                    i=6
                    waiting_list_flag=1
                    break

            print("[EDRONE1]:KEYS:{}".format(row_data_dict_keys))
            shared_dict[edrone1.rand_spwn.nth_row][2]=1

            #Setting nth_row i.e. y position
            nth_row_distance=((16-edrone1.rand_spwn.nth_row)*(-4))
            edrone1_sp[1][1]=nth_row_distance
            edrone1_sp[2][1]=nth_row_distance
            edrone1_sp[3][1]=nth_row_distance
            # index+=2

            #Setting x position
            edrone1_sp[1][0]=(shared_dict[edrone1.rand_spwn.nth_row][1])-3 #intermidiate x in edrone1.rand_spwn.nth_row
            edrone1_sp[2][0]=shared_dict[edrone1.rand_spwn.nth_row][1]-1  #updating lastly found x in edrone1.rand_spwn.nth_row
            # print("shared_dict",shared_dict)
            print("[edrone"+str(edrone1.drone_num)+"] working on row ", edrone1.rand_spwn.nth_row)



        pos.pose.position.x=edrone1_sp[i][0]
        pos.pose.position.y=edrone1_sp[i][1]
        pos.pose.position.z=edrone1_sp[i][2]

        if exception_flag==1:

            while (not edrone1.psMt.is_reached(pos,0.2,extra_precision_z)):
                if vel_flag and i==3:
                    # print('inside goto vel cmd')
                    vel.header.stamp=rospy.Time.now()
                    local_vel_pub.publish(vel)

                     # Failsafe mechanism when it reaches the end of a row
                    if edrone1.psMt.ps.pose.position.x>50:
                        i=6
                        setpoint_finished=0
                        shared_dict[edrone1.rand_spwn.nth_row][1]=0 #setting last seen  to zero because to find missed one
                        for i in waiting_list_boxes:
                            if edrone1.rand_spwn.nth_row==i[1]:
                                waiting_list_boxes.remove(i)
                                
                        continue
                    elif edrone1.psMt.ps.pose.position.x>25 and z_correcting_flag==1:
                        z_correcting_flag=0
                        rospy.logwarn("correcting z")
                        z_correction=edrone1.psMt.ps
                        z_correction.pose.position.z=3
                        for _ in range(45):
                            local_pos_pub.publish(z_correction)
                            edrone1.rate.sleep()

                    edrone1.img_aruco.update_aruco_position()

                    #Strategy (for minimizing the time of delivery and flying cost)
                    if (edrone1.img_aruco.aruco_position.pose.position.x!=0) and (edrone1.img_aruco.aruco_position.pose.position.y!=0) and (edrone1.img_aruco.aruco_position.pose.position.z!=0) and ( edrone1.img_aruco.aruco_position.pose.position.x-history>0.7):
                        rospy.logwarn("old History: {}".format(history))
                        rospy.loginfo('['+edrone1.drone_name+']: '+"Aruco detected")
                        history= edrone1.img_aruco.aruco_position.pose.position.x
                        rospy.logwarn("new history:{}".format(history))
                        
                        # Appends lesser priority boxes to the waiting list
                        if (edrone1.img_aruco.aruco_id==1) and (not (edrone1.rand_spwn.nth_row>6 or shared_dict[edrone1.rand_spwn.nth_row][0]==1)):

                            rospy.loginfo('red box appended')
                            
                            waiting_list_boxes.append([edrone1.img_aruco.aruco_position.pose.position.x,edrone1.rand_spwn.nth_row,edrone1.img_aruco.aruco_id])

                            rospy.loginfo("waiting list :{}".format(waiting_list_boxes))

                        elif (edrone1.img_aruco.aruco_id==2) and (not (edrone1.rand_spwn.nth_row<=6 or shared_dict[edrone1.rand_spwn.nth_row][0]==1)):

                            rospy.loginfo('blue box appended')
                            waiting_list_boxes.append([edrone1.img_aruco.aruco_position.pose.position.x,edrone1.rand_spwn.nth_row,edrone1.img_aruco.aruco_id])
                            rospy.loginfo("waiting list:{}".format(waiting_list_boxes))

                        else:
                            #Sets higher priority
                            rospy.loginfo(' out of red or blue box')
                            pos.pose.position.x= edrone1.img_aruco.aruco_position.pose.position.x
                            vel_flag=False

                        if (waiting_list_flag==0):
                            shared_dict[-1][0]-=1 #decreasing one box count
                            shared_dict[edrone1.rand_spwn.nth_row][0]-=1 #decreasing row box count 
                            shared_dict[edrone1.rand_spwn.nth_row][1]=edrone1.img_aruco.aruco_position.pose.position.x #upadating last aruco_box 

                        #Storing found aruco position
                        edrone1.img_aruco.aruco_position.pose.position.x=0
                        edrone1.img_aruco.aruco_position.pose.position.y=0
                        edrone1.img_aruco.aruco_position.pose.position.z=0

                        edrone1.vel_rate.sleep()

                else:
                    # print('inside goto pos cmd')
                    local_pos_pub.publish(pos)
                    edrone1.rate.sleep()
                # edrone1.img_aruco.img_show()

            edrone1_sp[i][0]=pos.pose.position.x
            edrone1_sp[i][1]=pos.pose.position.y
            edrone1_sp[i][2]=pos.pose.position.z


            # if edrone1_sp[i]!=last_sp:
            rospy.loginfo('['+ edrone1.drone_name+']: '+"Published setpoint {}".format(tuple(edrone1_sp[i])))
            rospy.loginfo("[edrone1]: i:{},spf:{},vel_flag:{}".format(i,setpoint_finished,vel_flag))
                # last_sp=edrone1_sp[i]

        # if i==3 and vel_flag==1:
           
            # rospy.loginfo('['+edrone1.drone_name+']: '+'velocity command')
            # edrone1_sp[3]=edrone1.psMt.goto(edrone1_sp[i],0,True)
      

        if setpoint_finished==3:
            # To get the drone on aruco
            # print(edrone1_sp[3])
            edrone1_sp[3][2]=-0.1
            vel_flag=0
            extra_precision_z=0.1
            setpoint_finished+=1
            destination_set_flag=1
            continue

        elif setpoint_finished==4:

            if(edrone1.gripperMt.status=="True"):
                gripper_failed=0
                rospy.loginfo('['+edrone1.drone_name+']: '+'Ready to activate gripper')
                sfsrv=False
                while(not sfsrv):
                    sfsrv=edrone1.gripperMt.gripper_action(True)
                    rospy.loginfo_once('['+edrone1.drone_name+']: '+'srsrv:{}'.format(sfsrv))
            else:
                # rospy.logwarn_once('['+edrone1.drone_name+']: '+'Gripper=false'
                gripper_failed+=1
                if (gripper_failed==700):
                    rospy.logwarn_once("[edrone1]: triggering gripper failed failsafe")
                    rospy.logwarn_once("edrone1_sp={}".format(edrone1_sp[2]))
                    i=2
                    setpoint_finished=2
                    vel_flag=1
                    history=-10
                    gripper_failed=0
                    shared_dict[edrone1.rand_spwn.nth_row][0]+=1
                continue

            #Setting destination truck
            if destination_set_flag:
                if edrone1.img_aruco.aruco_id == 1:  
                    dest_truck_base_x,dest_truck_base_y=red_truck_wrt_edrone1
                    detached_boxes_on_one_truck=detached_box_red
                    detached_box_red+=1
                
                else:   
                    dest_truck_base_x,dest_truck_base_y=blue_truck_wrt_edrone1
                    detached_boxes_on_one_truck=detached_box_blue
                    detached_box_blue+=1

                    
                #Stacking algorithm 
                if detached_boxes_on_one_truck%12==0: 
                    #resets m,n when detached boxes are 12 (i.e.) when two rows are filled with 2 boxes
                    m=-1
                    n=-1

                if detached_boxes_on_one_truck%6==0:
                    m=-1 #resets m when last column is filled with 2 boxes
                    n+=1 #increamenting n after filling last column

                if detached_boxes_on_one_truck%2==0:
                    m+=1 #increamenting m after stacking two boxes in a cell
            
                dest_truck_x=dest_truck_base_x+(n*(cell_length/3)) # incrementing by cell width every two boxes
                dest_truck_y=dest_truck_base_y+(m*(cell_width)) #toggling at every three detached boxes
                
                # assigning particular cell location to drop the box
                edrone1_sp[4][0:2]=dest_truck_x+0.85,dest_truck_y
                edrone1_sp[5][0:2]=dest_truck_x,dest_truck_y
                edrone1_sp[6][0:2]=dest_truck_x,dest_truck_y

                rospy.logwarn_once("dest loc:({},{})".format(dest_truck_x,dest_truck_y))

                destination_set_flag=0

        
            # rospy.loginfo('['+edrone1.drone_name+']: '+'Inside i==4')
            

            edrone1_sp[3][2]=3
            setpoint_finished+=1
            extra_precision_z=0

            continue

        elif setpoint_finished==7:
            #Detaching box from the drone
            rospy.loginfo('['+edrone1.drone_name+']: '+'Inside setpoint_finished==6')
            gripper_result=True
            while(gripper_result):
                gripper_result=edrone1.gripperMt.gripper_action(False)
            detached_boxes_on_two_trucks+=1

         

        if i==6 and waiting_list_flag==0:
            # resetting variables so that it performs tasks for the next strawberry box
            i=0
            setpoint_finished=0
            vel_flag=1
            edrone1_sp[1][2]=5
            edrone1_sp[3]=[50,nth_row_distance,3]
            z_correcting_flag=1
            shared_dict[edrone1.rand_spwn.nth_row][2]=0 #resetting working flag status
            history=-10

        if i==6 and  waiting_list_flag==1:
            print("waiting list")
           
            if token_num<len(waiting_list_boxes):
                i=2
                setpoint_finished=2
                vel_flag=0
                exception_flag=1
                edrone1_sp[3]=[waiting_list_boxes[token_num][0],(waiting_list_boxes[token_num][1]-16)*4,5]
                edrone1.img_aruco.aruco_id=waiting_list_boxes[token_num][2]
                token_num+=1
                print('incrementing token num to {}'.format(token_num))
           
        i=i+1
        setpoint_finished+=1

        if i>=(len(edrone1_sp)):
            rospy.loginfo('['+edrone1.drone_name+']: '+'AUTO LAND')
            edrone1.ofb_ctl.autoland_set_mode()
            break
        
if __name__=='__main__':

    try:
        """Main function"""
        manager=Manager() 
        shared_dict=manager.dict() #Shared dictionary 

        #Creating Drone instances
        edrone0=Drone(0)
        edrone1=Drone(1)

        #Assigning different cores for different drones (Multiprocessing)
        process0=Process(target=edrone0_main,args=(edrone0,shared_dict))
        process1=Process(target=edrone1_main,args=(edrone1,shared_dict))

        #Starting the processes
        process0.start()
        process1.start()
        
        #Joining the processes
        process0.join()
        process1.join()

    except rospy.ROSInitException as e:
       rospy.logerr_once(e)
