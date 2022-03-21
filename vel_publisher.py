#!/usr/bin/env python


#SCRIPT FOR READING IMAGE FROM CAMERA AND OBTAINING DISTANCE TO VEST


import rospy
from std_msgs.msg import String
import rospy
from sensor_msgs.msg import Image , PointCloud2
import sensor_msgs.point_cloud2
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


#Difference between UWB and RealSense distance admitted (m)
difference_threshold=2

#Measurement of the robot (m)
robot_measure=1.04
    #Future threshold:1.54

#Threshold in order to stop the robot when the operator is close (m)
stop_threshold=1.55


#Tolerance for the turning angle (rad-10 degrees)
angle_tolerance=0.15

#Proportional gain for velocity
kp=0.5

#-------------------------------------------------------------#FUNCTIONS-------------------------------------------------------------------------------------------------------------------------

def angularModule(pose):
    orientation = abs(pose)

    #Funciona bien
    #0.5 y #0.7
    if orientation<=(math.pi/2)/3:
        aV=0.6
    else:
        aV=0.7
    return aV

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



def talker():
    pub = rospy.Publisher('/robot/move_base/cmd_vel', Twist, queue_size=10)
    rospy.init_node('vel_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    robot_Vel=Twist()

    while not rospy.is_shutdown():

        distance_to_operator = rospy.get_param('/distance_to_operator')
        if distance_to_operator!=False:
            distance_to_operator=float(distance_to_operator)
        distance_to_anchor = float(rospy.get_param('/distance_to_anchor'))
        theta_odom = float(rospy.get_param('/theta_odom'))
        theta_anchor = float(rospy.get_param('/theta_anchor'))
       

        print("Camera distance: ",distance_to_operator,"Anchor distance: ",distance_to_anchor)
        print("Robot pose: ",theta_odom, "Anchor pose: ",theta_anchor)

    

        #When no operator is detected by camera-STOP    
        if distance_to_operator==False:
            print('NO OPERATOR DETECTED BY CAMERA-STOP')
            #Stop movement
            #Check movements in simulation
            robotVel_linear=0.0
            robotVel_angular= 0.0
        #When operator detected by camera matches with UWB
        elif abs(distance_to_operator-distance_to_anchor)<difference_threshold:
            if abs(distance_to_operator-distance_to_anchor)>0.3:
                distance_to_operator=distance_to_anchor
            else:
                distance_to_operator=distance_to_operator
            #If operator close - STOP
            if distance_to_operator <= stop_threshold:
                print('OPERATOR CLOSE-STOP')
                #Stop movement
                #Check movements in simulation
                robotVel_linear=0.0
                robotVel_angular= 0.0

            #If operator not close - MOVE
            else:
                print('Sending Movement command-Moving robot to Pose')

                #Get angular velocity if orientation is not okay
                if (theta_odom < theta_anchor-angle_tolerance) or (theta_odom > theta_anchor+angle_tolerance):
                    raw_pose=theta_anchor-theta_odom
                    aV=angularModule(raw_pose)
                    robotVel_angular= angularVelocity(theta_anchor,theta_odom,aV)
                else:
                    print("In range")
                    robotVel_angular=0.0
                
                #Only for checking
                if robotVel_angular>0:
                    print("Turning left")
                elif robotVel_angular<0:
                    print("Turning right")
              
                #Get linear velocity increasing proportional to the distance
                robotVel_linear= kp*(distance_to_operator-robot_measure)
                print("Moving forward")
    
    
        #When operator detected by camera does not match with UWB-STOP
        else:
            print('OPERATOR DETECTED BY CAMERA IS NOT OKAY-STOP')
            #Stop movement
            robotVel_linear=0.0
            robotVel_angular= 0.0

   

        robot_Vel.linear.x=robotVel_linear
        robot_Vel.angular.z=robotVel_angular
        
        
        pub.publish(robot_Vel)
        print(robot_Vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

