#!/usr/bin/env python


import message_filters, rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg  import Point
import os
import plotly.offline as go_offline
import plotly.graph_objects as go
from nav_msgs.msg import Odometry
import csv

class global_map:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """
        self.current_path=os.path.dirname(__file__)

        #CSV files
        with open(self.current_path+'/GPS_data.csv',mode='w') as self.gps_data:
            self.gps_data_writer=csv.writer(self.gps_data,delimiter=',',quotechar='"',quoting=csv.QUOTE_MINIMAL)
            self.gps_data_writer.writerow(["x (m)","y (m)","w (Kg)"])


    def listener(self):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
        rospy.init_node('mapmaker', anonymous=False)
    
  
        weigh_sub = message_filters.Subscriber('/Weigh_values', Float64)
        gps_sub = message_filters.Subscriber('/odom', Odometry)
        #gps_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry) 
        
        ts = message_filters.ApproximateTimeSynchronizer([weigh_sub, gps_sub], queue_size=10, slop=0.1, allow_headerless=True)
        
        ts.registerCallback(self.callback)

        # spin() simply keeps python from exiting until this node is stopped
    
        rospy.spin()


    def callback(self,weigh, gps):
        
        #Read weigh
        weigh=float(weigh.data)

        #Read odom data
        current_gps = gps


        #Get theta_odom from the robot Odometry
        x_gps = current_gps.pose.pose.position.x
        y_gps = current_gps.pose.pose.position.y
       

        with open(self.current_path+'/GPS_data.csv',mode='a') as self.gps_data_act:
            self.gps_data_writer_act=csv.writer(self.gps_data_act,delimiter=',',quotechar='"',quoting=csv.QUOTE_MINIMAL)
            self.gps_data_writer_act.writerow([x_gps,y_gps,weigh])
  

        #print("x: ",x_gps," y: ",y_gps," w: ",weigh)

        


    

if __name__ == '__main__':
    map=global_map()
    map.listener()

