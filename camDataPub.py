#!~/.virtualenvs/cv420/bin/python
# -*- coding: utf-8 -*-

"""
Author:  Henry Ugochukwu Odoemelem
Created: 4-May-2020 
"""

import cv2
import camV4L
import numpy as np
from threading import Thread
import camDT
import time
import rospy
import datetime
from std_msgs.msg import String


#                 0         1     2      3         4           5        6         7
trackerType = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
selectedTracker = 6
cv2.destroyAllWindows()
ImageSize = (544, 288)
cam = camV4L.camImage(source = 0,resolution = ImageSize)

contrastval = 20
exposureval = 54


cam.setContrast(contrastval) # 0 to 255
cam.setExposure(exposureval) # 3 to 2047

cv2.namedWindow   ('Camera Settings')
cv2.createTrackbar('Exposure',  'Camera Settings', exposureval,  2047, cam.setExposure)   # Sets the exposure 
cv2.createTrackbar('Contrast',  'Camera Settings', contrastval,  255, cam.setContrast)   # Sets the Contrast

newFrame = False
while not newFrame:
    try:
        newFrame,frameL,frameR = cam.read()
        cv2.imshow("Processed image frame", frameL)
    except:
        cam.StopCam()
        cv2.destroyAllWindows() 
      



exitloop1 = False
exitloop2 = False
exitloop3 = False

obj1 = camDT.camDetectandTrack(arucoID = 23,tractID=selectedTracker,newFrame = newFrame,frameL = frameL,frameR = frameR,Pl = cam.Pl,Pr = cam.Pr)
obj1.start()
obj2 = camDT.camDetectandTrack(arucoID = 50,tractID=selectedTracker,newFrame = newFrame,frameL = frameL,frameR = frameR,Pl = cam.Pl,Pr = cam.Pr)
obj2.start()
# obj3 = camDT.camDetectandTrack(arucoID = 70,tractID=selectedTracker,newFrame = newFrame,frameL = frameL,frameR = frameR,Pl = cam.Pl,Pr = cam.Pr)
# obj3.start()

countloop =0
while not exitloop1 and not exitloop2:# and not exitloop3: # only one id need to be available for loop to exit
   
    newFrame,frameL,frameR = cam.read()
    cv2.imshow("Processed image frame", frameL)
    exitloop1 = obj1.Run(newFrame,frameL,frameR)
    exitloop2 = obj2.Run(newFrame,frameL,frameR)
    # exitloop3 = obj3.Run(newFrame,frameL,frameR)
  

    if countloop >=20:
        cam.StopCam()
        cv2.destroyAllWindows() 
        obj1.exitcamDetectandTrack()
        obj2.exitcamDetectandTrack()
        # obj3.exitcamDetectandTrack()
        obj1.join()
        obj2.join()
        # obj3.join()
        break
    countloop = countloop + 1
    


outframeL1 = None
outframeR1 = None
outframeL2 = None
outframeR2 = None
outframeL3 = None
outframeR3 = None

fps =None
x1, y1, z1, x2, y2, z2,x3, y3, z3= 0,0,0,0,0,0,0,0,0

def talker():
    global x1,y1,z1
    global x2,y2,z2 
    global x3,y3,z3 
    countframe = 0
	
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
  

    fps = 0
    while not rospy.is_shutdown() and (not (cv2.waitKey(1) & 0xFF == ord('q'))):
        tic = time.perf_counter()
        newFrame,frameL,frameR = cam.read()

        if newFrame:
       
            outframeL1,outframeR1,outboxl1,outboxr1,x1,y1,z1 = obj1.startTracking(newFrame,frameL,frameR)
            outframeL,outframeR,outboxl2,outboxr2,x2,y2,z2 = obj2.startTracking(newFrame,outframeL1,outframeR1)
            # outframeL,outframeR,outboxl3,outboxr3,x3,y3,z3 = obj3.startTracking(newFrame,outframeL2,outframeR2)

          
            cv2.putText(outframeL, " fps : " + str(int(fps)), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
            
          
            # obj1.drawEpipolarLine(outframeL,outframeR,outboxl1,outboxr1)
            # obj2.drawEpipolarLine(outframeL,outframeR,outboxl2,outboxr2)
            # outframe = np.concatenate((outframeL,outframeR), axis=1)
            # cv2.imshow("Processed image frame", outframe) # display both image
            
           
            
       
            cv2.imshow("Processed image frame", outframeL) # display only left image

            fps = 1 / (time.perf_counter()  - tic ) # 1 frame divide by time for 1 frame 


            # data = "%s  \t %s \t %s \t %s \t %s \t %s\t %s \t %s \t %s " %(
            #             x1, y1, z1, x2, y2, z2, x3, y3, z3)

            data = "%s  \t %s \t %s \t %s \t %s \t %s" %(
                        x1, y1, z1, x2, y2, z2 )

            # data = "%s  \t %s \t %s \t %s " %(
            #         x1, y1, z1, )
            # rospy.loginfo(data)
            pub.publish(data)

            
            
        

if __name__ == '__main__':
    try:
        talker()
        cam.StopCam()
        cv2.destroyAllWindows() 
        obj1.exitcamDetectandTrack()
        obj2.exitcamDetectandTrack()
        # obj3.exitcamDetectandTrack()
        obj1.join()
        obj2.join()
        # obj3.join()
    except rospy.ROSInterruptException:
        cam.StopCam()
        cv2.destroyAllWindows() 
      

        obj1.exitcamDetectandTrack()
        obj2.exitcamDetectandTrack()
        # obj3.exitcamDetectandTrack()
        obj1.join()
        obj2.join()
        # obj3.join()
    except KeyboardInterrupt:
        cam.StopCam()
        cv2.destroyAllWindows() 
        obj1.exitcamDetectandTrack()
        obj2.exitcamDetectandTrack()
        # obj3.exitcamDetectandTrack()
        obj1.join()
        obj2.join()
        # obj3.join()

    
cam.StopCam()
cv2.destroyAllWindows() 


obj1.exitcamDetectandTrack()
obj2.exitcamDetectandTrack()
# obj3.exitcamDetectandTrack()
obj1.join()
obj2.join()
# obj3.join()


