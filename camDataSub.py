#!~/.virtualenvs/cv420/bin/python
# -*- coding: utf-8 -*-
"""
Author:  Henry Ugochukwu Odoemelem
Created: 4-May-2020 
"""

import serial
import math
from threading import Thread
import rospy
import time
import numpy as np
from std_msgs.msg import String 

ser = serial.Serial('/dev/ttyACM1',9600, timeout=5) # Ti MSP430

def inverse_Kinematics(ix,iy,EfH) :
    # values in cm 
    l1=1.12
    l2=12.5
    l3=9.0
    Base=5.5
    base_li=6.62#l1+Base

    try:
        iz=((EfH))-Base
        J1H=base_li-(EfH)
        s=math.sqrt(pow(ix,2) +pow(iy,2))
        r=math.sqrt(pow(s,2) +pow(J1H,2))
        alpha2=math.acos((pow(r,2)+pow(l3,2)-pow(l2,2))/(2*r*l3)) #cosine rule 
        psi=math.atan2(J1H,s)
        Efangle=psi+alpha2
        Efangle=2*math.pi-Efangle # orientation of the end effector
        iphi = Efangle

        Sth2=(iz-l1-(l3*math.sin(iphi)))/l2
        Cth2=math.sqrt(1-pow(Sth2,2))
        ith2=math.atan2(Sth2,Cth2)*(180/math.pi)
        
        Sth1=iy/(l3*math.cos(iphi)+l2*Cth2)
        Cth1=math.sqrt(1-pow(Sth1,2))
        
        ith1=math.atan2(Sth1,Cth1)*(180/math.pi)
        if ix<0:
            if ith1<180 and ith1>0:
                ith1=180-ith1 #to give equivalent of ith1 in obtuse form, since since x is negative
            if ith1==0: #because ith1 calculation will compute 180 as 0 even when x is negative because they are equivalent
                ith1=180
            
        ith3=(iphi*(180/math.pi))-ith2

    
        if ith1>180:
            ith1=360-ith1
        if ith2>180:
            ith2=360-ith2
        if ith3>180:
            ith3=360-ith3

            
        return int(ith1 ),int(ith2),int(ith3)


    except:
        pass



class sensorData:

    def __init__(self):
   
        self.pose = "  0 \t 0 \t 0 \t 1 \t 1 \t 1  "  
 
                    
    def position(self, data):
    
        self.pose = data.data
        return self.pose
    
    
 
def listener():
    
    # current arm position
    th1=90
    th2=90
    th3 =0
    
    #object positions
    x1,y1,z1 = 0,10,10
    x2,y2,z2 = 0,10,10

    y1old, y2old = 100,100
    
    # arm home initial angles for objects 23 and 50
    i23th1 ,i23th2,i23th3 = 0,90,90
    i50th1 ,i50th2,i50th3 = 180,90,90
    

    sensorData1 = sensorData()     
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, sensorData1.position)
    
    while not rospy.is_shutdown():
        jointData = sensorData1.pose        
        js= jointData.split("\t")

        x1 = -float(js[0])
        y1 = 21 - float(js[1]) #21 is length of L2+L3
        z1 = float(js[2])  + 1

        x2 = -float(js[3])
        y2 = 21 - float(js[4]) 
        z2 = float(js[5]) + 1
  
        try:
            if abs(abs(y1old)- abs(y1)) > 0.3: # do only is change is significant
                th1,th2,th3 = inverse_Kinematics(x1,y1,z1) 
                if i23th1 != th1 and i23th2 != th2 and i23th3 != th3 and th3 <= 90:
                    # time.sleep(0.5)
                    i23th1,i23th2,i23th3 = th1,th2,th3
                    rospy.loginfo("Object id 23, Joint angles in deg.: "+ str(i23th1)+" " +str(i23th2)+" " +str(i23th3))
                    rospy.loginfo("Object id 23 position in cm: "+ str(x1) +" " +str(y1)+" " +str(z1))
                    ser.write(bytearray([251,i23th1])) #loctate object 23 arm1 first
                    time.sleep(0.1)
                    ser.write(bytearray([252,i23th2,253,i23th3])) #loctate object 23
                    time.sleep(2)
                    ser.write(bytearray([251,0,252,90,253,90])) # rotate base to 0 to indicate id 23
                    time.sleep(1)
                    y1old = y1
                        
        except:
            print ("Error on object id 23 ")

        try:
            if abs(abs(y2old) - abs(y2)) > 0.3: # do only is change is significant
                th1,th2,th3 = inverse_Kinematics(x2,y2,z2) 
                if i50th1 != th1 and i50th2 != th2 and i50th3 != th3 and th3 <= 90:
                    # time.sleep(0.5)
                    i50th1,i50th2,i50th3 = th1,th2,th3
                    rospy.loginfo("Object id 50, Joint angles in deg.: "+ str(i50th1)+" " +str(i50th2)+" " +str(i50th3))
                    rospy.loginfo("Object id 50, position in cm "+ str(x2)+" " +str(y2)+" " +str(z2))
                    ser.write(bytearray([251,i50th1])) #locate object 50 arm1 first
                    time.sleep(0.1)
                    ser.write(bytearray([252,i50th2,253,i50th3])) #locate object 50
                    time.sleep(1)
                    ser.write(bytearray([251,180,252,90,253,90])) # rotate to base to 180 to indicate id 50
                    time.sleep(1)
                    y2old = y2
                
        except:
            print ("Error on object id 50")


      

if __name__ == '__main__':
    
    try:
       listener()
       ser.close()
       print("Ros shutdown, serial port closed")
    except rospy.ROSInterruptException:
       ser.close()
       print("Ros interrupeted, serial port closed")
    except KeyboardInterrupt:
       ser.close()
       print("KeyboardInterrupt, serial port closed")
