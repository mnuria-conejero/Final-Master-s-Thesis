#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import numpy as np
import rospy
from sensor_msgs.msg import Image , PointCloud2
import sensor_msgs.point_cloud2
from rospy.numpy_msg import numpy_msg
import cv2
import ros_numpy
import math
import os
import message_filters


def callback(color_data,depth_data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    

    folderC='/home/summit/Pictures/realsense/color/'
    folderD='/home/summit/Pictures/realsense/depth/'

    im_color = np.frombuffer(color_data.data, dtype=np.uint8).reshape(color_data.height, color_data.width, -1)
    im_depth = np.frombuffer(depth_data.data, dtype=np.uint8).reshape(depth_data.height, depth_data.width, -1)
    im_depth_mod=im_depth[:,:,1]
    
    im_color=cv2.cvtColor(im_color, cv2.COLOR_BGR2RGB)
    
    

    latest_file_indexC = len(os.listdir(folderC))
    filenameC=folderC + str(latest_file_indexC) + '.jpg'
    
    latest_file_indexD = len(os.listdir(folderD))
    filenameD=folderD + str(latest_file_indexD) + '.jpg'
    
    cv2.imwrite(filenameD,im_depth_mod)
    cv2.imwrite(filenameC,im_color)





def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Image_and_distance_reader', anonymous=True)
    
  
    im_color_subs=message_filters.Subscriber('/camera/color/image_raw', Image)
    im_depth_subs=message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    
    
    

    ts = message_filters.ApproximateTimeSynchronizer([im_color_subs,im_depth_subs], 1,1)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

