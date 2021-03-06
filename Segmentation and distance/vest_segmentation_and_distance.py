
#!/usr/bin/env python


#SCRIPT FOR READING IMAGE FROM CAMERA AND OBTAINING DISTANCE TO VEST


import rospy
from std_msgs.msg import String
import numpy as np
import rospy
from sensor_msgs.msg import Image , PointCloud2
import sensor_msgs.point_cloud2
import cv2
import math
import os
import message_filters
import cv_bridge
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from geometry_msgs.msg  import PoseStamped,Pose,Twist,TwistStamped
import tf
from nav_msgs.msg import Odometry

#-------------------------------------------#GLOBAL VARIABLES--------------------------------------------------------------------------------------------------------------------------

#Limits for HSV segmentation
    #All dataset images
lower_limit=(0.0041,0.5213,0.5266)
upper_limit=(0.1877,0.9990,0.9983)

    #Sunny day images
lower_limit_sun=(0.0037,0.5114,0.5113)
upper_limit_sun=(0.3402,0.9944,0.9969)

    #Cloudy day images
lower_limit_cloud=(0.000268,0,5077,0.7160)
upper_limit_cloud=(0.4216,0,9958,0.9978)

#-------------------------------------------------------------#FUNCTIONS-------------------------------------------------------------------------------------------------------------------------

#Function for segmenting the orange vest
#Inputs:
    #-image: RGB image from camera
    #-lower_limit: lower HSV coefficients obtained from GA
    #-upper_limit: upper HSV coefficients
#Outputs:
    #-result: segmented binary image processed
def colorSegmentation(image,lower_limit,upper_limit):
    image_hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    normalized_hsv = cv2.normalize(image_hsv, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    mask=cv2.inRange(normalized_hsv,lower_limit,upper_limit)
    #cv2.imshow('Original binary',mask)
    kernel=np.ones((5,5),np.uint8)
    erode=cv2.erode(mask,kernel)
    #cv2.imshow('Erosion1',erode)
    dilate=cv2.dilate(erode,kernel,iterations=4)
    #cv2.imshow('Dilate1',dilate)
    erode=cv2.erode(dilate,kernel)
    #cv2.imshow('Erosion2',erode)
    result=erode
    return result

#Function to calculate Distance of the bounding box from the depth image
#Inputs:
    #-segmented_result: output from ColorSegmentation()
    #-depth_array: image from camera converted to distance data
    #-y1,y2,x1,x2: coordinates of the bounding box
#Outputs:
    #-distance_to_vest: distance in m to the detected vest
def calculateDistance(segmented_result,depth_array,y1,y2,x1,x2):
    #Get the mean distance from the pixels of the bounding box
    mean_distance=0.0
    mean_counter=0.0
    for i in range(y1,y2):
        for j in range(x1,x2):
            if segmented_result[int(i),int(j)] == 255:
                mean_distance=mean_distance+(depth_array[int(i),int(j)])/1000
                mean_counter=mean_counter+1
        
    if mean_counter==0:
        distance_to_bb=None
    else:
        distance_to_bb=mean_distance/mean_counter
    return distance_to_bb

#Function for obtaining the distance to the vest segmented in ColorSegmentation()
#Inputs:
    #-segmented_result: output from ColorSegmentation()
    #-depth_image: depth image from camera
#Outputs:
    #-distance_to_vest: distance in m to the detected vest
def getDepthDistance(segmented_result,depth_image,raw_depth,distance_to_anchor):
    #Extract boundind boxes from the segmented result by looking at the contours
    contours=cv2.findContours(segmented_result,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=contours[0] if len(contours) == 2 else contours[1]
    
    min_difference=1000
    x,y,h,w=0,0,0,0
    x_draw,y_draw,h_draw,w_draw=0,0,0,0

    for idx,cntr in enumerate(contours):
        xt,yt,wt,ht=cv2.boundingRect(cntr)
        xt=int(xt)
        yt=int(yt)
        wt=int(wt)
        ht=int(ht)

        cv2.rectangle(segmented_result, (xt, yt), (xt+wt, yt+ht), (100, 0, 0), 2)
        size=wt*ht
        if size>4000:
            x,y,w,h=xt,yt,wt,ht
            x1=x
            x2=x+w
            y1=y
            y2=y+h

            #Check if the coordinates are lower or higher than dimensions
            xmax=depth_image.shape[1]#width
            ymax=depth_image.shape[0]#height
            xmin=0
            ymin=0
            if x1<xmin:
                x1=xmin
            if x1>xmax:
                x1=xmax
            if y1<ymin:
                y1=ymin
            if y1>ymax:
                y1=ymax
            if x2<xmin:
                x2=xmin
            if x2>xmax:
                x2=xmax
            if y2<ymin:
                y2=ymin
            if y2>ymax:
                y2=ymax
        
        
            # Use cv_bridge() to convert the ROS image to OpenCV format
            bridge = CvBridge()
            try:
                depth_image_array = bridge.imgmsg_to_cv2(raw_depth, desired_encoding="passthrough")
            except CvBridgeError as e:
                print(e)
        
        
            depth_array=np.array(depth_image_array, dtype=np.float32)
            distance_to_bb=calculateDistance(segmented_result,depth_array,y1,y2,x1,x2)

            if distance_to_bb==None or distance_to_bb==0.0:
                continue
            elif abs(distance_to_anchor-distance_to_bb)<min_difference:
                min_difference=distance_to_bb
                x_draw,y_draw,w_draw,h_draw=x,y,w,h
        else:
            continue

        

    if min_difference==1000:
        distance_to_vest=None
    else:
        distance_to_vest=min_difference
        cv2.rectangle(segmented_result, (x_draw, y_draw), (x_draw+w_draw, y_draw+h_draw), (255, 0, 0), 2)
        nameText=str(round(distance_to_vest,4)) + ' m'
        cv2.putText(segmented_result, nameText, (x_draw, y_draw-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,0,0), 2)
        cv2.imshow('Segmentation', segmented_result)
        

    return distance_to_vest
        
