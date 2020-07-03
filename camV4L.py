"""
Author:  Henry Ugochukwu Odoemelem
Created: 4-May-2020 
"""

import time
import struct
import subprocess
import cv2
import numpy as np
# logitech c525 properties via v4l2-ctl
#                     brightness (int)    : min=0 max=255 step=1 default=128 value=128
#                       contrast (int)    : min=0 max=255 step=1 default=128 value=128
#                     saturation (int)    : min=0 max=255 step=1 default=128 value=128
# white_balance_temperature_auto (bool)   : default=1 value=1
#                           gain (int)    : min=0 max=255 step=1 default=0 value=0
#           power_line_frequency (menu)   : min=0 max=2 default=2 value=2
#      white_balance_temperature (int)    : min=2000 max=6500 step=1 default=4000 value=2594 flags=inactive
#                      sharpness (int)    : min=0 max=255 step=1 default=128 value=128
#         backlight_compensation (int)    : min=0 max=1 step=1 default=0 value=0
#                  exposure_auto (menu)   : min=0 max=3 default=3 value=1
#              exposure_absolute (int)    : min=3 max=2047 step=1 default=250 value=333
#         exposure_auto_priority (bool)   : default=0 value=1
#                   pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
#                  tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
#                 focus_absolute (int)    : min=0 max=250 step=5 default=0 value=125
#                     focus_auto (bool)   : default=1 value=0
#                  zoom_absolute (int)    : min=100 max=500 step=1 default=100 value=100
#to see camera accepted resolutions:   --list-formats-ext
#
#e.gs of logitech c525 resolution; 640x480,160x120,176x144,320x176,320x240,422x240,352x288,544x288,640x360,752x416,800x448


class camImage():

    def __init__(self, source = 0, resolution = (544, 288)):

        self.source1 = source 
        self.source2 = source + 2
        
        self.camProperties = {'brightness':128, 'contrast': 25, 'saturation': 0,
             'gain': 23, 'sharpness': 0, 'exposure_auto': 1,
             'exposure_absolute': 1023, 'exposure_auto_priority': 0,
             'focus_auto': 0, 'focus_absolute': 0, 'zoom_absolute': 1,
             'white_balance_temperature_auto': 0, 'white_balance_temperature': 2800}


        for key in self.camProperties:
            subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(self.camProperties[key]))],
                            shell=True)
            subprocess.call(['v4l2-ctl -d /dev/video2 -c {}={}'.format(key, str(self.camProperties[key]))],
                            shell=True)

        self.cam1 = cv2.VideoCapture(self.source1,cv2.CAP_V4L2) # Use cv2.CAP_DSHOW if on a windows PC
        # self.cam1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self.cam1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) # trackbar works with this

        self.cam1.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.resolution = (int(self.cam1.get(cv2.CAP_PROP_FRAME_WIDTH)), 
                           int(self.cam1.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        
        self.cam2 = cv2.VideoCapture(self.source2,cv2.CAP_V4L2)
        # self.cam2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self.cam2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

       


        self.cam2.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.resolution = (int(self.cam2.get(cv2.CAP_PROP_FRAME_WIDTH)), 
                           int(self.cam2.get(cv2.CAP_PROP_FRAME_HEIGHT)))


        self.leftImage = None
        self.rightImage =None
        
     
        
       

        ######## Unrectified camera parameters for ImageSize = (544, 288)################################

        self.cMatrixr =np.array( [[439.59731982 ,  0.  ,       269.80055053],
                                [  0.  ,       439.22300607, 143.95737863],
                                [  0.    ,       0.  ,         1.        ]])

        self.cMatrixl = np.array( [[424.37377986  , 0.  ,       273.05342749],
                                    [  0.      ,   425.06630163 , 140.10688074],
                                    [  0.       ,    0.   ,        1.        ]])

        self.distCr =  np.array( [[ 0.07498266],
                                    [-0.18503787],
                                    [ 0.00111275],
                                    [ 0.00726558],
                                    [ 0.39932496]])

       
        self.distCl = np.array( [[ 0.05257172],
                                [-0.29232969],
                                [-0.00098365],
                                [ 0.00617034],
                                [ 0.07534596]])

        self.R = np.array([[ 0.99995187 ,-0.00837098 , 0.00511781],
                            [ 0.00853753 , 0.99940436 ,-0.03343718],
                            [-0.00483486 , 0.03347926,  0.99942772]])

        self.T= np.array( [-47.19772823, -0.9012592 , 9.28331359])


        ########### end  of Unrectified camera parameters for 544x288#################



        self.Rl, self.Rr, self.Pl, self.Pr, self.Q, self.roil, self.roir= cv2.stereoRectify(self.cMatrixl, self.distCl, self.cMatrixr, self.distCr,
                                                       self.resolution,self.R,self.T,None, None, None, None, None,flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
        self.mapxl, self.mapyl = cv2.initUndistortRectifyMap(self.cMatrixl,  self.distCl, self.Rl, self.Pl,self.resolution,cv2.CV_32FC1)
        self.mapxr, self.mapyr = cv2.initUndistortRectifyMap(self.cMatrixr,  self.distCr, self.Rr, self.Pr,self.resolution,cv2.CV_32FC1)
        
 


    def read(self):
       
        if(self.cam1.isOpened() and self.cam2.isOpened()):
 
            rval1, self.leftImage = self.cam1.read()
            rval2, self.rightImage = self.cam2.read()
            
            if(rval1 and rval2):
                
                #Comment out for camera calibartion
                self.leftImage = cv2.remap(self.leftImage, self.mapxl, self.mapyl, cv2.INTER_LINEAR ) # rectified left image
                self.rightImage = cv2.remap(self.rightImage, self.mapxr, self.mapyr, cv2.INTER_LINEAR ) # rectified right image
                
              
                return True, self.leftImage, self.rightImage
            else:
              return False, None
        
        else:
            return False, None

    def getPl(self):
        return self.Pl
    def getPr(self):
        return self.Pr
    def StopCam(self):
        self.cam1.release()
        self.cam2.release()
    def setExposure(self, exposure):

        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format('exposure_absolute', str(max(3, exposure)))],
                        shell=True)
        subprocess.call(['v4l2-ctl -d /dev/video2 -c {}={}'.format('exposure_absolute', str(max(3, exposure)))],
                        shell=True)

    
    def setContrast(self, contrast):

        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format('contrast', str(contrast) )],
                        shell=True)
        subprocess.call(['v4l2-ctl -d /dev/video2 -c {}={}'.format('contrast', str(contrast) )],
                        shell=True)


