#!/usr/bin/env python

from turtle import color
import rospy, time, os, sys, random
import serial
import serial.tools.list_ports as ports
from std_msgs.msg       import Float64
import matplotlib.pyplot as plt

plt.style.use('ggplot')

#plt.style.use('dark_background')
#plt.style.use('seaborn')
#plt.style.use('seaborn-bright')
#plt.style.use('seaborn-ticks')
#plt.style.use('seaborn-dark')

import numpy as np


class read_weigh:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """

        # Init node
        rospy.init_node('Weigh_reader', anonymous=False)

       # Get port and tag name
        #serial_number='0'
        #pts=ports.comports()


        '''try:

            for p in pts:
                if p.serial_number == serial_number:
                    self.weigh_port='/devices/pci0000:00/0000:00:08.1/0000:06:00.4/usb3/3-2/3-2:1.0/0003:04D8:0055.0017/input/input49'
        except:
            print("Please connect the weighing device")'''

        ''
        
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port settings
        '''self.serialPortWeigh = serial.Serial(
            port = self.weigh_port,
            baudrate = 9600,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )'''

        #Create File to write data
        #self.f = open("weigh_data.txt","w+")
        self.weigh=0.0

        #Create the empty arrays for the dynamic plot
        self.line1=[]
        self.x_data=[]
        self.x_data_all=[]
        self.y_data_all=[]
        self.y_data=[]
        self.counter=0
    

    def main(self) :
        """
        Initialize port 
        :param:
        :returns: none
        """

        # close the serial port in case the previous run didn't closed it properly
        #self.serialPortWeigh.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        #self.serialPortWeigh.open()

        # check if the serial port is opened
        #if(self.serialPortWeigh.isOpen()):
            #rospy.loginfo("Port opened: "+ str(self.serialPortWeigh.name) )
            # start initialization of the weigh by making an initial tare
        self.initializeWeigh()
            # give some time to weigh to wake up

        self.initseconds = rospy.get_time()
        time.sleep(2)
            
        rospy.loginfo("Reading Weigh")
        #else:
            #rospy.loginfo("Can't open port: "+ str(self.serialPortWeigh.name))

        try:

            while not rospy.is_shutdown():
                # just read everything from serial port

                #python2
                #serialReadLine = self.serialPortWeigh.read_until()

                #python3
                #serialReadLine= self.serialPortWeigh.read_until().decode()

                #Keyboard entry
                serialReadLine=str(raw_input())
                serialReadLine.replace(',', '.')

                try:
                    self.publishWeighValues(serialReadLine)

                except IndexError:
                    rospy.loginfo("Found index error in the network array! DO SOMETHING!")



        except KeyboardInterrupt:
            rospy.loginfo("Quitting Weigh and closing port, allow 1 second for recovery")
            #self.f.close() #Close txt file
            #self.serialPortWeigh.reset_input_buffer()

        finally:
            rospy.loginfo("Quitting")
            #self.f.close() #Close txt file
            #self.serialPortWeigh.reset_input_buffer()
            #self.rate.sleep()

            #python2
            #serialReadLine = self.serialPortWeigh.read_until()

            #python3
            #serialReadLine= self.serialPortWeigh.read_until().decode()

            #Keyboard entry
            serialReadLine=str(raw_input())
            serialReadLine.replace(',', '.')

            rospy.loginfo("succesfully closed ")
            #self.serialPortWeigh.close()


    def publishWeighValues(self, serialData):
        """
        Publish Weigh Values
        :param networkDataArray:  Array from serial port containing all information
        :returns: none
        """
        Data=''

        arrayData = [x.strip() for x in serialData.strip()]


        for data in arrayData:
            if data == '(' or data==')':
                Data=Data
            elif data ==',':
                Data=Data + '.'
            else:
                Data = Data + data
        
        
        
        #Data = serialData.strip()
        try:
            #Keyboard input
            Data=float(Data)

            #Create Publisher
            weigh_pub = rospy.Publisher('/Weigh_values', Float64, queue_size=100)
        except:
            pass


        
        try :

            #Publish topic and write in file
            #weigh = float(Data[0])
           
            #Keyboard Input
            if abs(self.weigh-float(Data))>=0.12:
                self.weigh = float(Data)
            else:
                self.weigh=self.weigh
            #self.f.write("Weigh: "+ str(weigh)+ " Kg" + '\n')

            weigh_pub.publish(self.weigh)


            #Add data to the plot arrays
            seconds = rospy.get_time()-self.initseconds
            if len(self.y_data) > 100:
                self.x_data.pop(0)
                self.y_data.pop(0)

                self.x_data.append(seconds)
                self.y_data.append(self.weigh)

                self.x_data_all.append(seconds)
                self.y_data_all.append(self.weigh)
            else:
                self.x_data.append(seconds)
                self.y_data.append(self.weigh)

                self.x_data_all.append(seconds)
                self.y_data_all.append(self.weigh)
            #Update plots
            self.counter=self.counter+1
            print(self.counter)
            self.line1=self.live_plotter(self.x_data,self.y_data,self.line1)

            
        except :
            pass


                
    def initializeWeigh(self):
        """
        Initialize weigh, by making a tare and create a file to write results
        :param:
        :returns: none
        """

        # Send_tare
        #self.serialPortWeigh.write('t')
        
        # sleep for half a second
        time.sleep(0.5)

    #Adding point by point
    def live_plotter(self,x_vec,y1_data,line1,identifier='',pause_time=0.01):

        if line1==[]:
            #Allowing dynamic plot
            plt.ion()
            fig=plt.figure(figsize=(13,6))
            ax=fig.add_subplot(111)
            plt.xticks(fontsize=14)
            plt.yticks(fontsize=14)

            #Create a variable for the line in order to update it later
            line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)

            #Update plot label/title
            ylabel_obj=plt.ylabel('Weigh (Kg)',fontsize=18)
            xlabel_obj=plt.xlabel('Time (s)',fontsize=18)
            plt.setp(ylabel_obj, color='k') 
            plt.setp(xlabel_obj,color='k')
            title_obj=plt.title('Weigh in time'.format(identifier),fontsize=24)
            plt.setp(title_obj, color='b')   
            plt.show


        #Adjust limits when entering new data
        if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
            plt.ylim([-0.1,np.max(y1_data)+1])

        if np.min(x_vec)<=line1.axes.get_xlim()[0] or np.max(x_vec)>=line1.axes.get_xlim()[1]:
            plt.xlim([np.min(x_vec),np.max(x_vec)+0.5])

        #Pause the data in order to catch up
        plt.pause(pause_time)
        
        
        #If all is already created, just update
        line1.set_data(x_vec,y1_data)

        

        #This is in order to update it in the next iteration
        return line1
    


if __name__ == '__main__':
    try:
        weigh = read_weigh()
        weigh.main()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass