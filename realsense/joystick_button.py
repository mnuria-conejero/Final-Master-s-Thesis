#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, Joy
from geometry_msgs.msg import Twist
import message_filters
import sensor_msgs.point_cloud2
from rospy.numpy_msg import numpy_msg
import cv2
import ros_numpy
import math
import rospy
import cv2
import cv_bridge
import os


def callback(im_data,depth_data,button_data):
    global record
    buttons=button_data.buttons
    if buttons[11]==1:
	print('Esta pulsado el R3')
	record=1
    if buttons[10]==1:
        print('Esta pulsado el L3')
	record=0
    if buttons[10]==0 and buttons[11]==0:
	print('No esta pulsado ni el R3 ni el L3')
    if record==1:
        print('Estamos guardando imagenes')
  	im_color = np.frombuffer(im_data.data, dtype=np.uint8).reshape(im_data.height, im_data.width, -1)
	im_color=cv2.cvtColor(im_color, cv2.COLOR_BGR2RGB)
	depth = np.frombuffer(depth_data.data, dtype=np.uint8).reshape(depth_data.height, depth_data.width, -1)
        depth_mod=depth[:,:,1]
        folder='/home/summit/Pictures/realsense/depth/'
	folderC='/home/summit/Pictures/realsense/color/'
        latest_file_index = len(os.listdir(folder))
	latest_file_indexC = len(os.listdir(folderC))
	filename=folder + str(latest_file_index) + '.jpg'
	cv2.imwrite(filename,depth_mod)
	filenameC=folderC + str(latest_file_indexC) + '.jpg'
	cv2.imwrite(filenameC,im_color)

    #print(data.buttons)
    
global record
#global id
#id=0
record=0
def joy_record():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_record', anonymous=True)

    # rospy.Subscriber('/camera/color/image_raw', Image, callback)
    # rospy.Subscriber('/camera/depth/points', PointCloud2, callback_points)
    # rospy.Subscriber('/kinect2/sd/points', PointCloud2, callback_pointsK)
    # rospy.Subscriber('/kinect2/hd/image_color', Image, callback_K)
    print('teke')
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    # points_sub = message_filters.Subscriber('/kinect2/sd/image_depth', Image)
    button_sub = message_filters.Subscriber('/robot/joy', Joy)
    #rospy.Subscriber('/robot/joy', Joy, callback)
    # image_sub = message_filters.Subscriber('/robot/front_rgbd_camera/rgb/image_raw', Image)
    # points_sub = message_filters.Subscriber('/robot/front_rgbd_camera/depth/points', PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub,depth_sub, button_sub], 1, 1)
    #ts = message_filters.TimeSynchronizer([image_sub, button_sub], 5)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    joy_record()
