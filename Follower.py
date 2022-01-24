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
from geometry_msgs.msg  import PoseStamped,Pose,Twist
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

#Difference between UWB and RealSense distance admitted (m)
difference_threshold=1

#Measurement of the robot (m)
robot_measure=1.04
    #Future threshold:1.54

#Threshold in order to stop the robot when the operator is close (m)
stop_threshold=1.64

#Angular Velocity stablished for turning
aV=0.25

#Tolerance for the turning angle (rad-5º)
angle_tolerance=0.09

#Proportional gain for velocity
kp=0.4

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



#Function for obtaining the distance to the vest segmented in ColorSegmentation()
#Inputs:
    #-segmented_result: output from ColorSegmentation()
    #-depth_image: depth image from camera
#Outputs:
    #-distance_to_vest: distance in m to the detected vest
def getDepthDistance(segmented_result,depth_image,raw_depth):
    #Extract boundind boxes from the segmented result by looking at the contours
    contours=cv2.findContours(segmented_result,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=contours[0] if len(contours) == 2 else contours[1]
    #Comparing all the contours to find out the biggest one
    max=0
    x,y,h,w=0,0,0,0
    for idx,cntr in enumerate(contours):
        xt,yt,wt,ht=cv2.boundingRect(cntr)
        xt=int(xt)
        yt=int(yt)
        wt=int(wt)
        ht=int(ht)
        size=wt*ht
        if size>max:
            max=size
            x,y,w,h=xt,yt,wt,ht
    
    cv2.rectangle(segmented_result, (x, y), (x+w, y+h), (255, 0, 0), 2)
    cv2.imshow('Segmentation', segmented_result)
    
    if w*h > 4000: #If the BB area is > 4000 it is considered valid
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
        #Get the mean distance from the pixels of the bounding box
        mean_distance=0.0
        mean_counter=0.0
        for i in range(y1,y2):
            for j in range(x1,x2):
                mean_distance=mean_distance+(depth_array[int(i),int(j)])/1000
                mean_counter=mean_counter+1
        
        distance_to_vest=mean_distance/mean_counter
        return distance_to_vest
    else:
        return None


#Function for getting the message in order to stop robot
#Inputs:
    #-None
#Outputs:
    #-Stop message: message to set all velocities to 0
def stoppingRobot():
    stop_message=Twist()
    stop_message.linear.x=0.0
    stop_message.linear.y=0.0
    stop_message.linear.z=0.0
    stop_message.angular.x=0.0
    stop_message.angular.y=0.0
    stop_message.angular.z=0.0
    return stop_message


#Function for obtaining angular velocity in order to make the turn needed
#Inputs:
    #-theta_sensor: operator orientation
    #-theta_odom: orientation of the robot
    #-aV: module of the angular velocity
#Outputs:
    #factor*aV: module and direction of the angular Velocity

#-1: clockwise
# 1: counterclockwise
def angularVelocity(theta_sensor,theta_odom,aV):

    if theta_odom>0:
        if theta_sensor<=0:
            factor=-1
        elif theta_sensor>0:
            if theta_odom>theta_sensor:
                factor=-1
            elif theta_odom<theta_sensor:
                factor=1

    elif theta_odom<0:
        if theta_sensor>=0:
            factor=1
        elif theta_sensor<0:
            if abs(theta_odom)>abs(theta_sensor):
                factor=1
            elif abs(theta_odom)<abs(theta_sensor):
                factor=-1
    elif theta_odom==0:
        if theta_sensor==0:
            factor=0
        elif theta_sensor>0:
            factor=1
        elif theta_sensor<0:
            factor=-1
    
    return factor*aV


#-------------------------------------------------------------------#MAIN----------------------------------------------------------------------------------------------------------------------

def callback(color_data,depth_data,anchor_data,odom_data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    
    #Read images from topics
    im_color = np.frombuffer(color_data.data, dtype=np.uint8).reshape(color_data.height, color_data.width, -1)
    im_depth = np.frombuffer(depth_data.data, dtype=np.uint8).reshape(depth_data.height, depth_data.width, -1)

    #Read data from anchor
    anchorPose=anchor_data

    #Read odom data
    current_odometry = odom_data
    


    #Segment Image and Get the distance to vest from camera
    image_segmented_all=colorSegmentation(im_color,lower_limit,upper_limit)
    image_segmented_sun=colorSegmentation(im_color,lower_limit_sun,upper_limit_sun)
    image_segmented_cloud=colorSegmentation(im_color,lower_limit_cloud,upper_limit_cloud)
    image_segmented1=cv2.add(image_segmented_all,image_segmented_sun)
    image_segmented=cv2.add(image_segmented1,image_segmented_cloud)

    distance_to_operator=getDepthDistance(image_segmented,im_depth,depth_data)

    #Get anchor distance
    distance_to_anchor=anchorPose.pose.position.z

    #We create the velocity publisher
    nav_pub = rospy.Publisher("/robot/move_base/cmd_vel", Twist, queue_size=10)

    #When no operator is detected by camera-STOP
    if distance_to_operator==None:
        print('NO OPERATOR DETECTED BY CAMERA')
        #Stop movement
        #Check movements in simulation
        #stop_cmd=stoppingRobot()
        #nav_pub.publish(stop_cmd)

    #When operator detected by camera matches with UWB
    elif abs(distance_to_operator-distance_to_anchor)<difference_threshold:
        #If operator close - STOP
        if distance_to_operator <= stop_threshold:
            print('OPERATOR CLOSE')
            #Stop movement
            #Check movements in simulation
            #stop_cmd=stoppingRobot()
            #nav_pub.publish(stop_cmd)

        #If operator not close - MOVE
        else:
            print('Sending Movement command-Moving robot to Pose')

            #Get orientation from operator
            theta_sensor=anchorPose.pose.orientation.z
            
            #Get theta_odom from the robot Odometry
            orientation = current_odometry.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            euler = tf.transformations.euler_from_quaternion(orientation_list)
            #When there is no velocity in y axis
            theta_odom = euler[2]

            print("Theta_sensor: ",theta_sensor," Theta_odom",theta_odom)

            robotVel=Twist()

            #Turn around z axis toward operator orientation
            if (theta_odom < theta_sensor-angle_tolerance) or (theta_odom > theta_sensor+angle_tolerance):
                robotVel.linear.x = 0.0
                robotVel.angular.z = angularVelocity(theta_sensor,theta_odom,aV)
            #When orientation is OK move forward with velocity increasing proportional to the distance
            else:
                robotVel.linear.x = kp*(distance_to_operator-robot_measure)
                robotVel.angular.z = 0



            print(robotVel)

            
            #Check movements in simulation
            #nav_pub.publish(robotVel)
    
    #When operator detected by camera does not match with UWB-STOP
    else:
        print('OPERATOR DETECTED BY CAMERA IS NOT OKAY')
        #Stop movement
        #stop_cmd=stoppingRobot()
        #nav_pub.publish(stop_cmd)

    #waits for user to press any key 
    #(this is necessary to avoid Python kernel form crashing)
    cv2.waitKey(1) 
  
    #closing all open windows 
    #cv2.destroyAllWindows() 


#---------------------------------------------------------#TOPICS RECEIVED-------------------------------------------------------------------------------------------------------------------------------

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Image_and_distance_reader', anonymous=True)
    
  
    im_color_subs=message_filters.Subscriber('/camera/color/image_raw', Image)
    im_depth_subs=message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    anchor_orientation=message_filters.Subscriber('/dwm1001/orientation/to/anchor/', PoseStamped)
    odom=message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    
    

    ts = message_filters.TimeSynchronizer([im_color_subs,im_depth_subs,anchor_orientation,odom], 10)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

