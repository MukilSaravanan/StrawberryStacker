#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import aruco_library as aruco_library
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from task_1.msg import Marker
class image_proc():
    def __init__(self):
        rospy.init_node('marker_detection')
        self.marker_pub=rospy.Publisher('/marker_info',Marker,queue_size=1)
        self.image_sub=rospy.Subscriber('/camera/camera/image_raw',Image,self.image_callback)
        self.img=np.empty([])
        self.bridge=CvBridge()
        self.marker_msg=Marker()
    def image_callback(self,data):
        self.img=self.bridge.imgmsg_to_cv2(data,'bgr8')
        temp=aruco_library.detect_ArUco(self.img)
        cv2.imshow("Dotted Image",self.img)
        
        if temp==1:
            print(1)
            return
        self.marker_msg.id=int(list(temp.keys())[0])
        corners=temp[self.marker_msg.id][0]
        self.marker_msg.x=(corners[0][0] + corners[2][0])/2 
        self.marker_msg.y=(corners[1][1]+corners[3][1])/2
        self.marker_msg.yaw=aruco_library.Calculate_orientation_in_degree(temp)[self.marker_msg.id]
    def timer_callback(self,data):
        self.marker_pub.publish(self.marker_msg)
    def listener(self):
        timer=rospy.Timer(rospy.Duration(0.1),self.timer_callback)
        rospy.loginfo("Outside callback")
        rospy.spin()
        timer.shutdown()
    
if __name__=='__main__':
    image_proc_obj=image_proc()
    image_proc_obj.listener()