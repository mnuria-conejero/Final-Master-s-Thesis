#!/usr/bin/env python
""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""



import serial
import rospy, time, os, sys, random
import serial.tools.list_ports as ports

import numpy as np
import math

from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg       import Float64
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy import signal 


#python2

from dwm1001_apiCommands import DWM1001_API_COMMANDS


class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """

        # Init node
        rospy.init_node('DWM1001_Active_{}'.format(random.randint(0,100000)), anonymous=False)

        # Get port and tag name
        serial_number_izq='000760096513'
        serial_number_dech='000760096423'
        pts=ports.comports()
        try:

            for p in pts:
                if p.serial_number == serial_number_izq:
                    self.dwm_port_Tag_Izq=p.device
                elif p.serial_number == serial_number_dech:
                    self.dwm_port_Tag_Dech=p.device
        except:
            print("Please connect both UWB devices")
            
                
        
        
        #self.tag_name = rospy.get_param('~tag_name')
        self.tag_Izq_name = 'Tag_izq'
        self.tag_Dech_name = 'Tag_dech'
        self.use_network = rospy.get_param('~use_network', False)
        self.network = rospy.get_param('~network', "default")
        self.verbose = rospy.get_param('~verbose', False)

        #Initiate the counter for the median filter and the number of nmeasurements
        self.median_counter=0
        self.median_measurements=5

        #Initiate the median array
        self.median_left = []
        self.median_right= []
        #Distance between Tags
        self.distance_bt_tags=0.47 #m
        self.distance_to_middle=0.308  #m


        ''' #Kalman Filter parameters
        self.f = KalmanFilter (dim_x=2, dim_z=2)
        self.f.x = np.array([0., 0.])
        self.f.F = np.array([[1.,0.],[0.,1.]])
        #Medicion del sensor
        self.f.H = np.array([[1.,0.],[0.,1.]])
        #Matriz de covarianza (identidad de las dimensiones * uncertainty)
        self.f.P = np.array([[250.,    0.],[   0., 250.] ])
        #Ruido sensor--estaba en 0.1 y 0.13
        self.f.R = np.array([[5.,0.],[0.,5.]])
        self.f.Q = Q_discrete_white_noise(dim=2, dt=1., var=1.)'''


        #Low pass filter-order
        self.order = 2
        self.cut = 0.5
        self.b_filter, self.a_filter = signal.butter(self.order, self.cut, 'lowpass')
        
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port  Tag Izq settings
        self.serialPortDWM1001_Tag_Izq= serial.Serial(
            port = self.dwm_port_Tag_Izq,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

        # Serial port Tag Dech settings
        self.serialPortDWM1001_Tag_Dech = serial.Serial(
            port = self.dwm_port_Tag_Dech,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
    

    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001_Tag_Izq.close()
        self.serialPortDWM1001_Tag_Dech.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001_Tag_Izq.open()
        self.serialPortDWM1001_Tag_Dech.open()

        # check if the serial port 1is opened
        if(self.serialPortDWM1001_Tag_Izq.isOpen() and self.serialPortDWM1001_Tag_Dech.isOpen()):
            rospy.loginfo("Ports opened: "+ str(self.serialPortDWM1001_Tag_Izq.name) + " and " + str(self.serialPortDWM1001_Tag_Dech.name))
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()


            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 distance")
        else:
            rospy.loginfo("Can't open ports: "+ str(self.serialPortDWM1001_Tag_Izq.name) + str(self.serialPortDWM1001_Tag_Dech.name)) 

        try:

            while not rospy.is_shutdown():
                # just read everything from serial ports
                #python2
                #serialReadLine_Tag_izq = self.serialPortDWM1001_Tag_Izq.read_until()
                #serialReadLine_Tag_dech = self.serialPortDWM1001_Tag_Dech.read_until()
                
                #python3
                serialReadLine_Tag_izq = self.serialPortDWM1001_Tag_Izq.read_until().decode()
                serialReadLine_Tag_dech = self.serialPortDWM1001_Tag_Dech.read_until().decode()

                try:
                    self.publishTagPositions(serialReadLine_Tag_izq,serialReadLine_Tag_dech)
                    

                except IndexError:
                    rospy.loginfo("Found index error in the network array! DO SOMETHING!")



        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            # self.serialPortDWM1001.reset_input_buffer()
            self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            
            #python2
            #serialReadLine_Tag_izq = self.serialPortDWM1001_Tag_Izq.read_until()
            #serialReadLine_Tag_dech = self.serialPortDWM1001_Tag_Dech.read_until()

            #python3
            serialReadLine_Tag_izq = self.serialPortDWM1001_Tag_Izq.read_until().decode()
            serialReadLine_Tag_dech = self.serialPortDWM1001_Tag_Dech.read_until().decode()
            if "reset" in serialReadLine_Tag_izq and serialReadLine_Tag_dech:
                rospy.loginfo("succesfully closed ")
                self.serialPortDWM1001_Tag_Izq.close()
                self.serialPortDWM1001_Tag_Dech.close()


    def publishTagPositions(self, serialData_izq, serialData_dech):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """
        
        arrayData_izq = [x.strip() for x in serialData_izq.strip().split(',')]
        arrayData_dech = [x.strip() for x in serialData_dech.strip().split(',')]
        

        # If getting a tag position
        if "DIST" in (arrayData_izq[0] and  arrayData_dech[0]):
        
           
            # The number of elements should be 2 + 6*NUMBER_OF_ANCHORS + 5 (TAG POS)
            number_of_anchors = 1

            for i in range(number_of_anchors) :

                node_id_izq = arrayData_izq[2+6*i]
                node_id_dech = arrayData_dech[2+6*i]
                first_time = False
                if node_id_izq not in self.topics and node_id_dech not in self.topics:
                    first_time = True

                    #Creating publisher
                    
                    self.topics["Anchor pose"] = rospy.Publisher(
                        '/dwm1001/orientation/to/anchor/' ,
                        PoseStamped, 
                        queue_size=10
                    )

                
                #Main loop for obtaining the robot pose and distance to person
                    #First we filter the measurements and take the average measuraments every 10 iterations
                    #Then we compute the data and publish
                try :
                    dist_izq = float(arrayData_izq[7+6*i])
                    dist_dech = float(arrayData_dech[7+6*i])
                    print("Dist_izq: ",dist_izq," | Dist_dech: ",dist_dech)


                    #Add measurements to the arrays
                    self.median_left.append(dist_izq)
                    self.median_right.append(dist_dech)
                    self.median_counter=self.median_counter+1



                    if self.median_counter==self.median_measurements:


                        

                        print(self.median_left,self.median_right)

                        #Estimation of distance with lowpass_filter
                        
                        filtered_measure_izq = signal.filtfilt(self.b_filter, self.a_filter, [self.median_left],padlen=self.median_measurements-1)
                        filtered_measure_dech = signal.filtfilt(self.b_filter, self.a_filter, [self.median_right],padlen=self.median_measurements-1)

                        print("Filtered: ",filtered_measure_izq,filtered_measure_dech)

                        #Get average from filtered measurements
                        average_measure_izq=np.sum(filtered_measure_izq)/self.median_measurements
                        average_measure_dech=np.sum(filtered_measure_dech)/self.median_measurements

                        print("Average",average_measure_izq,average_measure_dech)
                        

                        '''#Estimation of the distances by means of the Kalman Filter
                        average_measure_izq,average_measure_dech=self.kalmanPrediction(average_measure_izq,average_measure_dech)

                        print("Kalman: ",average_measure_izq,average_measure_dech)'''
                       
                        #Compute and publish
                        x,y,theta,dist_r=self.calculate_RobotPose(average_measure_izq,average_measure_dech,self.distance_bt_tags,self.distance_to_middle)
                        
                        
                        p = PoseStamped()
                        p.header.stamp = rospy.Time.now()  
                        p.pose.position.x = x               #m
                        p.pose.position.y = y               #m
                        p.pose.position.z = dist_r            #m
                        p.pose.orientation.x = 0.0
                        p.pose.orientation.y = 0.0
                        p.pose.orientation.z = theta
                        p.pose.orientation.w = 0.0       #rad

                        self.topics["Anchor pose"].publish(p)


                        #print("Position: ",x,y)
                        #print("Angle_beforeKalman: ",theta_pre)
                        print("Angle: ",theta*57.29578)
                        print("dist: ",dist_r)

                        #Set counter to 0 and clean array
                        self.median_counter=0
                        self.median_left=[]
                        self.median_right=[]

                       
                    else: 
                        continue

                except :
                    pass

    '''def kalmanPrediction(self,measure_izq,measure_dech):
        z = np.array([measure_izq, measure_dech])
        self.f.predict()
        self.f.update(z)
        predicted_izq=self.f.x[0]
        predicted_dech=self.f.x[1]
        return predicted_izq,predicted_dech'''

    def calculate_RobotPose(self,d1_izq,d2_dech,d_bt_tags,d_to_middle):
        #Check if Q1 or Q2
        if d1_izq >= d2_dech:
            #Q1
            try:
                theta1=math.acos((d1_izq**2 + d_bt_tags**2 - d2_dech**2)/(2*d1_izq*d_bt_tags))
                #theta2=math.pi-math.acos((d2_dech**2 + d_bt_tags**2 - d1_izq**2)/(2*d2_dech*d_bt_tags))
                xa=d1_izq*math.sin(theta1)-d_to_middle
                ya=-1*(d1_izq*math.cos(theta1)-(d_bt_tags/2))
                thetar=math.atan(abs(xa)/abs(ya))
                robot_theta=-1*((math.pi/2)-thetar)
                #Calculate the distance from the back of the robot, not the middle 
                #This is in order to compare it with the camera distance
                dr=math.sqrt((xa+self.distance_to_middle)**2+ya**2)
            except ValueError:
                theta1=0.0
                xa=0.0
                ya=0.0
                thetar=0.0
                robot_theta=0.0
                dr=0.0

        else:              
            print("Q2")
            #Q2
            try:
                theta1=math.acos((d1_izq**2 + d_bt_tags**2 - d2_dech**2)/(2*d1_izq*d_bt_tags))
                #theta2=math.pi-math.acos((d2_dech**2 + d_bt_tags**2 - d1_izq**2)/(2*d2_dech*d_bt_tags))
                xa=d1_izq*math.sin(math.pi-theta1)-d_to_middle
                ya=d1_izq*math.cos(math.pi-theta1)+(d_bt_tags/2)
                thetar=math.pi-math.atan(abs(xa)/abs(ya))
                robot_theta=thetar-(math.pi/2)
                dr=math.sqrt((xa+self.distance_to_middle)**2+ya**2)
            except ValueError:
                theta1=0.0
                xa=0.0
                ya=0.0
                thetar=0.0
                robot_theta=0.0
                dr=0.0


        #Coordinates axis:
            #x positive: vertical forward
            #y positive: left forward
        x=xa
        y=ya

        #Theta axis
            #0 degrees: front 
            #90 degrees: left
            #-90 degrees: right
        theta=robot_theta
        dist_t=dr

        return x,y,theta,dist_t

 

        



                
    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.RESET)
        self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        self.serialPortDWM1001_Tag_Izq.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        self.serialPortDWM1001_Tag_Dech.write(DWM1001_API_COMMANDS.SINGLE_ENTER)





if __name__ == '__main__':
    try:
        dwm1001 = dwm1001_localizer()
        dwm1001.main()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
