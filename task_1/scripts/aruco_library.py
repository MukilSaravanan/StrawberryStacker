#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############
#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):

	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the       		aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}

	gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250 )
	parameters=aruco.DetectorParameters_create()
	corners,ids,_=aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

	if ids is None: #if aruco is not detected
		return 1



	for i in range(len(ids)):
		Detected_ArUco_markers[ids[i][0]]=corners[i]

	return Detected_ArUco_markers



def Calculate_orientation_in_degree(Detected_ArUco_markers):

	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}
	
	ArUco_marker_angles ={}

	for key in list(Detected_ArUco_markers.keys()):
		corners=Detected_ArUco_markers[key][0]
		cX = (corners[0][0] + corners[2][0]) / 2.0
		cY = (corners[1][1] + corners[3][1]) / 2.0
		center=[cX,cY]

	# To calculate midpoint
		midpoint_top_X=(corners[0][0]+corners[1][0])/2
		midpoint_top_Y=(corners[0][1]+corners[1][1])/2
		midpoint_top=[midpoint_top_X,midpoint_top_Y]

		diff_X=(midpoint_top[0]-center[0])
		diff_Y=-1*(midpoint_top[1]-center[1])
		
		if (diff_X>0 and diff_Y>0):
				angle_radian = math.atan2(abs(diff_Y),abs(diff_X))
				angle_degree = math.degrees(angle_radian)
		elif (diff_X<0 and diff_Y>0):
			angle_radian = math.atan2(abs(diff_Y),abs(diff_X))
			angle_degree = 180-math.degrees(angle_radian)
		elif (diff_X<0 and diff_Y<0):
			angle_radian = math.atan2(abs(diff_Y),abs(diff_X))
			angle_degree = 180+math.degrees(angle_radian)
				
					
		else:
				angle_radian = math.atan2(abs(diff_Y),abs(diff_X))
				angle_degree = 360-math.degrees(angle_radian)	
		
		
		
		ArUco_marker_angles[key]=angle_degree
		
		
	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionar


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

 	
	for key in list(Detected_ArUco_markers.keys()):
		
		corners=Detected_ArUco_markers[key][0]
		cX = int((corners[0][0] + corners[2][0]) / 2.0)
		cY = int((corners[1][1] + corners[3][1]) / 2.0)
		center=(cX,cY)
		midpoint_top_X=int(corners[0][0]+corners[1][0])//2
		midpoint_top_Y=int(corners[0][1]+corners[1][1])//2
		midpoint_top=(midpoint_top_X,midpoint_top_Y)
		cv2.circle(img,(corners[0][0],corners[0][1]),6,(125,125,125),-1)
		cv2.circle(img,(corners[1][0],corners[1][1]),6,(0,255,0),-1)
		cv2.circle(img,(corners[2][0],corners[2][1]),6,(180,105,255),-1)
		cv2.circle(img,(corners[3][0],corners[3][1]),6,(255,255,255),-1)
		cv2.circle(img,center,6,(0,0,255),-1)
		cv2.line(img,center,(midpoint_top),(255,0,0),4)
		


		font=cv2.FONT_HERSHEY_SIMPLEX
		img=cv2.putText(img,"{}".format(key),(cX+20,cY+20),font,1,(0,0,255),4)
		img=cv2.putText(img,"{}".format(math.ceil(ArUco_marker_angles[key])),(cX-30,cY-30),font,1,(0,255,0),4)
		
	return img
	
    



