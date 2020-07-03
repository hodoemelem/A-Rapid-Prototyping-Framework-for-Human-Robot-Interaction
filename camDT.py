"""
Author:  Henry Ugochukwu Odoemelem
Created: 4-May-2020 
"""

import cv2
import sys
import math
import numpy as np
from cv2 import aruco
import threading

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)



class camDetectandTrack(threading.Thread):
    
    def __init__(self,arucoID = -1,tractID = 6,newFrame = False,frameL = None,frameR = None,Pl = None, Pr = None):
        threading.Thread.__init__(self)
   
        self.Pl = Pl
        self.Pr = Pr
        self.Currentid = arucoID
        self.frameL = frameL
        self.frameR = frameR
        self.newFrame = newFrame
        self.tractID = tractID
        self.trackerType = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
        self.exitloop = False
        self.trackerL = cv2.TrackerCSRT_create()
        self.trackerR = cv2.TrackerCSRT_create()
        self.selectedTracker = self.trackerType[self.tractID]
 

        self.expand = 10 # margin to create tracking frame
        # BBMF = BBTF + TFMF
        # TFMF = BBMF - expand, for the x axis
        # BBMF = BOUNDING BOX WRT. MAIN FRAME
        # BBTF = BOUNDING BOX WRT. TRACKING FRMAE
        # TFMF = TRACKING FRAME WRT. MAIN FRAME
        self.TFMFL = (0, 0, 0, 0)
        self.TFMFR = (0, 0, 0, 0)
        
        self.BBMFL = (0, 0, 0, 0) 
        self.BBMFR = (0, 0, 0, 0)

        self.BBTFL = (0, 0, 0, 0)
        self.BBTFR = (0, 0, 0, 0)


        self.linesr = None
        self.linesl = None
      
        self.Xfocusl = 0
        self.Yfocusl = 0
        self.idsR =[0]
        self.idsL =[0]
        self.xcorr0= 0
        self.ycorr0= 0
        self.xcorr2 = 0
        self.ycorr2= 0
        self.xcorl0= 0 
        self.ycorl0= 0
        self.xcorl2= 0
        self.ycorl2= 0                            
        self.Rcorners =np.array([0,0,0,0])
        self.Lcorners =np.array([0,0,0,0])
        self.X_distance=0
        self.Y_distance=0
        self.Z_distance=0
        self.found = False
     
        self.okL = False
        self.okR = False
      
        self.p1L = (0, 0)
        self.p2L = (0, 0)
        self.Accu = 1009
        self.p1R = (0, 0)
        self.p2R = (0, 0)

        self.count = 0
        self.forbiddenmark = (84,144) # 272,144 is centre #84,144 fbid
        self.imgcenter = (272,144)
        self.imgLKp = np.array([[ 0],[0]],dtype=np.float)
        self.imgRkp = np.array([[0],[ 0]],dtype=np.float)
        self.points = np.array([0,0,0,0])
        self.X_distance= np.round(0 , 1)  
        self.Y_distance= np.round(0, 1) 
        self.Z_distance= np.round(0, 1) 
        self.x = 0
        self.y = 0
        self.z = 0
        
        self.Xfocusr = 0
        self.Yfocusr = 0

        #Fundamental matrix
        self.F = np.array( [[ 0 , 0 , 0],
                           [0 , 0 ,-1],
                           [0, 1  ,0]] )
        self.myLcorner = [0]
        self.myRcorner = [0]

        self.frameLtemp = None 
        self.frameRtemp = None

        self.distL = None
        self.distR = None

        self.blur = None

        self.threshl = None
        self.contoursL= None
        self.iL=0
        self.areaL = []
        self.maxareaL = None
        self.totalAreaL = None

        self.threshr = None
        self.contoursR= None
        self.iR=0
        self.areaR = []
        self.maxareaR = None
        self.totalAreaR = None

        self.image = None
        self.line  = None
        self.a=None
        self.b=None
        self.c=None
        
        self.x0=None #starting x point equal to zero
        self.x1=None #ending x point equal to the last column of the image

        self.y0= None#corresponding y points
        self.y1= None

   
        
        self.image=None
    
  
  


    def exitcamDetectandTrack(self):

        print("Exitcam")
      


    def run(self):
        pass
      
      
     
    def Run(self,newFrame,frameL,frameR):
        self.frameLtemp = self.frameL.copy()
        self.frameRtemp = self.frameR.copy()
        print("Starting tracker, waiting for detection...")
    
       
        if self.newFrame:
            
            print("still detecting")
            self.checkDist = False
            self.detectFrame()

        if self.exitloop == True:
            print("Tracker Started")
            return self.exitloop
        else:
            return False
           


    def startTracking(self,newFrame,frameL,frameR):
        self.newFrame,self.frameL,self.frameR = newFrame,frameL,frameR
        self.count = (self.count+1)%10
        if self.newFrame:
            # backup the main frame before processing starts
            self.frameLtemp = self.frameL.copy()
            self.frameRtemp = self.frameR.copy()
            
            try: 
                #crop image to size of Tracking frame
                self.frameL = self.frameL[int(self.TFMFL[1]):int(self.TFMFL[1] + self.TFMFL[3]), int(self.TFMFL[0]):int(self.TFMFL[0] + self.TFMFL[2])]
                self.frameR = self.frameR[int(self.TFMFR[1]):int(self.TFMFR[1] + self.TFMFR[3]), int(self.TFMFR[0]):int(self.TFMFR[0] + self.TFMFR[2])]
                
                if self.found == False:
                 
                    self.detectFrame() # Detect the object by searching the entire frame, if obeject was not found by tracker in previous cycle
           
             
                # Update tracker
                self.okL,self.BBTFL = self.trackerL.update(self.frameL)
                self.okR,self.BBTFR = self.trackerR.update(self.frameR)
                

                
                if self.okL and self.okR and self.found == True:
                    
                 

                    # getting new  size of tracking frame   realtive to the main frame, used for cropping
                    
                    self.TFMFL = (self.BBTFL[0] +self.TFMFL[0] -self.expand,self.BBTFL[1] +self.TFMFL[1]- self.expand,self.BBTFL[2] + self.expand*2,self.BBTFL[3] + self.expand*2 )
                    self.TFMFR = (self.BBTFR[0] +self.TFMFR[0]- self.expand,self.BBTFR[1] +self.TFMFR[1]- self.expand,self.BBTFR[2] + self.expand*2,self.BBTFR[3] + self.expand*2 )

                    
                    if self.count == 0: # do this periodically
                        try: # here , if object being tracked has been removed, reset tracking    
                            
                            # grab bounding box of object and make it grayscale
                            self.blur = cv2.cvtColor(self.frameL[int(self.BBTFL[1]):int(self.BBTFL[1] + self.BBTFL[3]), int(self.BBTFL[0]):int(self.BBTFL[0] + self.BBTFL[2])], cv2.COLOR_BGR2GRAY)
                           
                            #  blur the image
                            self.blur = cv2.GaussianBlur(self.blur,(5,5),0)
                            # apply adaptivethreshold so that image is black and white and contours are created
                            self.threshl = cv2.adaptiveThreshold(self.blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
                            # find the contours
                            self.contoursL, hierarchy = cv2.findContours(self.threshl, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                            
                            
                            self.iL=0
                            self.areaL = []
                            for cont in self.contoursL: 
                                self.areaL.append(cv2.contourArea(cont)) # append all contour areas to a list
                                self.iL=self.iL+1 # count number of contours
                            
                            # get the max area from the list, which is usaully the bounding box eges, it is an outliner
                            self.maxareaL = max(self.areaL) 
                            self.areaL.pop(self.areaL.index(self.maxareaL)) # remove the max area from the list of areas
                            self.totalAreaL = sum(self.areaL)               # sum the areas of the counters
                            
                            ## in case you want to view the contours, uncomment below code block
                            #self.threshl = cv2.cvtColor(self.threshl, cv2.COLOR_GRAY2BGR)
                            #cv2.drawContours(self.threshl, contours, -1, (0,255,0), 3)
                            #cv2.imshow("threshl", threshl)
                            #if no tracked object, totalArea is usually very small << 100
                            
                            if self.iL <5 or  self.totalAreaL < 25 : 
                                self.resetTracker()
                            
                            else:
                                # DO THE SAME THING FORM RIGHT IMAGE
                                self.blur = cv2.cvtColor(self.frameR[int(self.BBTFR[1]):int(self.BBTFR[1] + self.BBTFR[3]), int(self.BBTFR[0]):int(self.BBTFR[0] + self.BBTFR[2])], cv2.COLOR_BGR2GRAY)
                                self.blur = cv2.GaussianBlur(self.blur,(5,5),0)
                                self.threshr = cv2.adaptiveThreshold(self.blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

                                
                                self.contoursR, hierarchy = cv2.findContours(self.threshr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                
                                
                                self.iR=0
                                self.areaR = []
                                for cont in self.contoursR:
                                    self.areaR.append(cv2.contourArea(cont))
                                    self.iR=self.iR+1
                                
                                self.maxareaR = max(self.areaR)
                                self.areaR.pop(self.areaR.index(self.maxareaR))
                                self.totalAreaR = sum(self.areaR)
                                
                                ## in case you want to view the contours, uncomment below code block
                                #self.threshr = cv2.cvtColor(self.threshr, cv2.COLOR_GRAY2BGR)
                                #cv2.drawContours(self.threshr, contours, -1, (0,255,0), 3)
                                #cv2.imshow("threshr", self.threshr)

                                #if no tracked object, totalArea is usually very small <<100
                                # if the condition below is passed, then reset the tracker as object has been removed
                                if self.totalAreaL < 25 or self.totalAreaR <25: 
                                    self.resetTracker()
                               
                                

                        
                        except:
                            self.resetTracker()
                        

                    # size of bounding box relative to Main Frame
                    self.BBMFL = (self.BBTFL[0]+self.TFMFL[0],self.BBTFL[1]+self.TFMFL[1],self.BBTFL[2],self.BBTFL[3])
                    self.BBMFR = (self.BBTFR[0]+self.TFMFR[0],self.BBTFR[1]+self.TFMFR[1],self.BBTFR[2],self.BBTFR[3])
                    
                    self.Xfocusl = int(self.BBMFL[0]+ self.BBMFL[2]/2)
                    self.Yfocusl = int(self.BBMFL[1]+ self.BBMFL[3]/2)

                    self.Xfocusr = int(self.BBMFR[0]+ self.BBMFR[2]/2)
                    self.Yfocusr = int(self.BBMFR[1]+ self.BBMFR[3]/2)
                    
                    #--- Triangulation---
                    # focus midpoint on left image
                    self.imgLKp = np.array([[ self.Xfocusl],[self.Yfocusl]],dtype=np.float)
                    # focus midpoint on right image
                    self.imgRkp = np.array([[ self.Xfocusr],[self.Yfocusr]],dtype=np.float)
                    # object position in world corordinate
                    self.points = cv2.triangulatePoints(self.Pl,self.Pr,self.imgLKp,self.imgRkp)
                    self.X_distance= np.round(9+(self.points[0]/self.points[3])/10 , 1)  #division by 10 to convert to cm, division by fourth point because of homogenous coordinate
                    self.Y_distance= np.round( 0.3-(self.points[1]/self.points[3])/10 , 1)  #division by 10 to convert to cm   
                    self.Z_distance= np.round(44.5 - ((self.points[2]/self.points[3])/10)*(44.5/40), 1) #56 height from camera to platform
                    # self.Z_distance= np.round( ((self.points[2]/self.points[3])/10), 1) #56 height from camera to platform
                    # self.X_distance is a lsingle element list, assign the element to a variable
                    self.x = self.X_distance[0]
                    self.y = self.Y_distance[0]
                    self.z = self.Z_distance[0]

                    #use the main frame for display
                    self.frameL = self.frameLtemp.copy()
                    self.frameR = self.frameRtemp.copy()
            
                    self.p1L = (int(self.BBMFL[0]), int(self.BBMFL[1])) # x,y of 1st point of bounding box
                    self.p2L = (int(self.BBMFL[0] + self.BBMFL[2]), int(self.BBMFL[1] + self.BBMFL[3])) # x,y of point opposite to first point of bounnding box
                    cv2.rectangle(self.frameL, self.p1L, self.p2L, (0,255,0), 2, 1)  # draw bounding box on left image
                    cv2.circle(self.frameL,(int(self.BBMFL[0]+ self.BBMFL[2]/2),int(self.BBMFL[1]+ self.BBMFL[3]/2)),5, (0,255,0),-1) # draw dircle on left image
                    # self.frameL = cv2.circle(self.frameL,self.forbiddenmark ,5,(0,0,255),-1)#make forbidden region(object wiil not be seen by L/R camera) on 640 x 480 image 
                    # self.frameL = cv2.circle(self.frameL,self.imgcenter ,5,(0,0,255),-1)#mark forbidden region
                 
                    # self.p1R = (int(self.BBMFR[0]), int(self.BBMFR[1]))
                    # self.p2R = (int(self.BBMFR[0] + self.BBMFR[2]), int(self.BBMFR[1] + self.BBMFR[3]))
                    # cv2.rectangle(self.frameR, self.p1R, self.p2R, (0,255,0), 2, 1)
                    # cv2.circle(self.frameR,(int(self.BBMFR[0]+ self.BBMFR[2]/2),int(self.BBMFR[1]+ self.BBMFR[3]/2)),5,(0,255,0),-1)

                    # write the position of the marker midpoint next to the marker
                    cv2.putText(self.frameL,"ID"+str(self.Currentid) +"; "+ "X:" + str(self.x)+"Y:" + str(self.y)+"Z:" + str(self.z), (self.p1L[0],self.p1L[1]-5), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0,255,0), 1)
                    

                    self.checkDist = False
                    self.found = True      # object was found
                
                   
                    return self.frameL,self.frameR,self.BBMFL,self.BBMFR,self.x,self.y,self.z 
                 
                else : # obeject is not found, reset tracker and parameters
                    self.resetTracker()
                    self.frameL = self.frameLtemp.copy()
                    self.frameR = self.frameRtemp.copy()
                    self.fps = 0
                    self.BBMFL = (0, 0, 0, 0)
                    self.BBMFR = (0, 0, 0, 0)
                    # inform user of faliure
                    cv2.putText(self.frameL,"ID " + str(self.Currentid) + " NOT FOUND", (10,40+self.Currentid), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255),1)
                    # self.frameL = cv2.circle(self.frameL,self.forbiddenmark ,5,(0,0,255),-1)#mark forbidden region
                    # self.frameL = cv2.circle(self.frameL,self.imgcenter ,5,(0,0,255),-1)#mark forbidden region
                 
                    return self.frameL,self.frameR,self.BBMFL,self.BBMFR,self.x,self.y,self.z  
                
           
            except:# obeject is not found, reset tracker and parameters
                self.frameL = self.frameLtemp.copy()
                self.frameR = self.frameRtemp.copy()
                self.fps = 0
                self.BBMFL = (0, 0, 0, 0)
                self.BBMFR = (0, 0, 0, 0)
                return self.frameL,self.frameR,self.BBMFL,self.BBMFR,self.x,self.y,self.z  
              

    def getBox(self): # return object bounding box
        return self.BBMFL,self.BBMFR
    def getPosition(self): # get position of object midpoint
        return self.x,self.y,self.z

    

    def drawLine(self,image,line):
        # For drawing epipolar lines
        self.image = image
        self.line  = line
        self.a=line[0]
        self.b=line[1]
        self.c=line[2]
        
        self.x0=0 #starting x point equal to zero
        self.x1=self.image.shape[1] #ending x point equal to the last column of the image

        self.y0= int((-self.a*self.x0-self.c)/self.b) #corresponding y points
        self.y1= int((-self.a*self.x1-self.c)/self.b)

        #draw the line
        self.image= cv2.line(image,(self.x0,int(self.y0)),(self.x1,int(self.y1)),(0,0,255),1)
        n=np.array([self.x0, self.y0, 1])# points on x img  used to draw line on y img 
        
        return self.image,n
    
    def drawEpipolarLine(self,inframeL,inframeR,inBBMFL,inBBMFR):
   
        self.frameL,self.frameR,self.BBMFL,self.BBMFR = inframeL,inframeR,inBBMFL,inBBMFR
        
        self.myLcorner[0] = np.array([(int(self.BBMFL[0]+ self.BBMFL[2]/2)), (int(self.BBMFL[1]+ self.BBMFL[3]/2)), 1])
        self.myRcorner[0] = np.array([(int(self.BBMFR[0]+ self.BBMFR[2]/2)), (int(self.BBMFR[1]+ self.BBMFR[3]/2)), 1])

        #computing epipolar line on right image
        for i in range(1):#myLcorner[0] , draw only one point
            self.linesr=self.F.dot(self.myLcorner[i]) # lr=F*ql
            self.frameR,n= self.drawLine(self.frameR, self.linesr)   
            #myRcorner[0] = n # getting img points on right image to use in for loop under   
        #computing epipolar line on left image
        for i in range(1):
            self.linesl=np.matmul(self.F.transpose(), self.myRcorner[i])  #ll=F^T * qr
            self.frameL,_= self.drawLine(self.frameL,self.linesl)


    def resetTracker(self):
        # Tracking failure, reset
        self.selectedTracker = "KCF" # change tracker type
        self.trackerL = cv2.TrackerKCF_create() # create ne instace of tracker
        ok = self.trackerL.init(self.frameL, (0,0,0,0)) # initialize tracker with arbitary point
        img = np.ones((512,512,1), np.uint8)*255 # create white image
        self.okL, self.BBTFL = self.trackerL.update(img) # make it tracker point on white image
        self.okL, self.BBTFL = self.trackerL.update(self.frameL) # then make it track point on given frame, this will force the tracker to stop tracking 

        # Tracking failure,  reset
        self.trackerR = cv2.TrackerKCF_create()
        ok = self.trackerR.init(self.frameR, (0,0,0,0))
        img = np.ones((512,512,1), np.uint8)*255
        self.okR, self.BBTFR = self.trackerR.update(img)
        self.okR, self.BBTFR = self.trackerR.update(self.frameR)
        self.found = False

    def detectFrame(self):
        try:
            self.idsR =[0]
            self.idsL =[0]
            self.Lcorners,self.idsL,_ = cv2.aruco.detectMarkers(self.frameLtemp.copy() ,aruco_dict)
            self.Rcorners,self.idsR,_ = cv2.aruco.detectMarkers(self.frameRtemp.copy() ,aruco_dict)
            if self.idsL is None or self.idsR is None:
                self.idsR =[0]
                self.idsL =[0]
        except:
           pass

        if self.Currentid in self.idsR  and  self.Currentid in self.idsL:
            self.frameL = self.frameLtemp.copy() 
            self.frameR = self.frameRtemp.copy() 
            
            #unpack the corners
            for ptsr,ptsl in zip(([self.Rcorners[list(self.idsR).index(self.Currentid)]]),([self.Lcorners[list(self.idsL).index(self.Currentid)]])):
                pass
                
            self.xcorr0= (ptsr[:,0])[:,0] 
            self.ycorr0= (ptsr[:,0])[:,1]

            self.xcorr2 = (ptsr[:,2])[:,0] 
            self.ycorr2= (ptsr[:,2])[:,1]

    
            self.xcorl0= (ptsl[:,0])[:,0] 
            self.ycorl0= (ptsl[:,0])[:,1]

            self.xcorl2= (ptsl[:,2])[:,0] 
            self.ycorl2= (ptsl[:,2])[:,1]
            
       
            # give four corners of the object and get a bounding box cordinate (x_tl, y_tl, w, h)
            [xl, yl, wl, hl] = cv2.boundingRect(np.array([[self.xcorl0,self.ycorl0],[self.xcorl2,self.ycorl2],[(ptsl[:,1])[:,0] ,(ptsl[:,1])[:,1]], [(ptsl[:,3])[:,0] ,(ptsl[:,3])[:,1]] ]))
            [xr, yr, wr, hr] = cv2.boundingRect(np.array([[self.xcorr0,self.ycorr0],[self.xcorr2,self.ycorr2],[(ptsr[:,1])[:,0] ,(ptsr[:,1])[:,1]], [(ptsr[:,3])[:,0] ,(ptsr[:,3])[:,1]] ]))
            
    
            # Define an initial bounding box
            self.BBMFL = (xl, yl, wl, hl) # (x_tl, y_tl, w, h)
            self.BBMFR = (xr, yr, wr, hr) # (x_tl, y_tl, w, h)

        
            # Initialize tracker with first frame and bounding box
            self.selectedTracker = self.trackerType[self.tractID]

        
        
            if self.selectedTracker == 'MOSSE':
                self.trackerL = cv2.TrackerMOSSE_create()
                self.trackerR = cv2.TrackerMOSSE_create()
            if self.selectedTracker == "CSRT":
                self.trackerL = cv2.TrackerCSRT_create()
                self.trackerR = cv2.TrackerCSRT_create()
            if self.selectedTracker == 'BOOSTING':
                self.trackerL = cv2.TrackerBoosting_create()
                self.trackerR = cv2.TrackerBoosting_create()
            if self.selectedTracker == 'MIL':
                self.trackerL = cv2.TrackerMIL_create()
                self.trackerR = cv2.TrackerMIL_create()
            if self.selectedTracker == 'KCF':
                self.trackerL = cv2.TrackerKCF_create()
                self.trackerR = cv2.TrackerKCF_create()
            if self.selectedTracker == 'TLD':
                self.trackerL = cv2.TrackerTLD_create()
                self.trackerR = cv2.TrackerTLD_create()
            if self.selectedTracker == 'MEDIANFLOW':
                self.trackerL = cv2.TrackerMedianFlow_create()
                self.trackerR = cv2.TrackerMedianFlow_create()
            if self.selectedTracker == 'GOTURN':
                self.trackerL = cv2.TrackerGOTURN_create()
                self.trackerR = cv2.TrackerGOTURN_create()

            
            #  size of tracking frame  realtive to the main frame
            self.TFMFL = (self.BBMFL[0] - self.expand,self.BBMFL[1] - self.expand,self.BBMFL[2] + self.expand*2,self.BBMFL[3] + self.expand*2 )
            self.TFMFR = (self.BBMFR[0] - self.expand,self.BBMFR[1] - self.expand,self.BBMFR[2] + self.expand*2,self.BBMFR[3] + self.expand*2 )

            #Crop image to tracking frame only
            self.frameL = self.frameL[int(self.TFMFL[1]):int(self.TFMFL[1] + self.TFMFL[3]), int(self.TFMFL[0]):int(self.TFMFL[0] + self.TFMFL[2])]
            self.frameR = self.frameR[int(self.TFMFR[1]):int(self.TFMFR[1] + self.TFMFR[3]), int(self.TFMFR[0]):int(self.TFMFR[0] + self.TFMFR[2])]
            
            # BBTFL size bounding box relative to Tracking frame
            self.BBTFL = (self.BBMFL[0]-self.TFMFL[0],self.BBMFL[1]-self.TFMFL[1],self.BBMFL[2],self.BBMFL[3])
            self.BBTFR = (self.BBMFR[0]-self.TFMFR[0],self.BBMFR[1]-self.TFMFR[1],self.BBMFR[2],self.BBMFR[3])
    

            self.okL = self.trackerL.init(self.frameL, self.BBTFL)
            self.okR = self.trackerR.init(self.frameR, self.BBTFR)

            
        
            self.exitloop = True
            self.found = True
        
          