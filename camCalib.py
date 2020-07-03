"""
Author:  Henry Ugochukwu Odoemelem
Created: 4-May-2020 
"""


import cv2
import math
import numpy as np
import camV4L
import time
"""
I recommend running camCalib on a windows PC

"""
############### Uncomment this code block to take chessboard images ###################################################
# #####FOR TAKING SAMPLE IMAGES
resolution = (544, 288)
cam = camV4L.camImage(source = 0,resolution = resolution )


cv2.namedWindow   ('Settings')
cv2.createTrackbar('Exposure',  'Settings', 523,  2047, cam.setExposure)   # Sets the exposure 
cv2.createTrackbar('Contrast',  'Settings', 20,  255, cam.setContrast)   # Sets the Contrast

path="/home/pi/Desktop/chesspics544x288/"
counter  = 0
def snapShot():
    global counter
    fps = 0 
    while(True):
        tic = time.perf_counter()
    
        newFrame, imgL,imgR = cam.read()
       
        if newFrame:
 
            cv2.putText(imgL, " fps : " + str(int(fps)), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
            
            img = np.concatenate((imgL, imgR), axis=1)
            cv2.imshow(str(counter)+' Left and Right Images',img)
   
        
        if cv2.waitKey(1) & 0xFF == ord('a'):
            cv2.imwrite(path+ str(counter)+'chessL.jpg',imgL)
            cv2.imwrite(path+str(counter)+'chessR.jpg',imgR)
            print (str(counter) + " Image Saved.")
            counter = counter + 1
            cv2.destroyAllWindows()
        fps = 1 / (time.perf_counter()  - tic )

if __name__ == '__main__':
    try:
        snapShot()
    except:
        print("camera error")
        pass


## When everything done, release the capture
cam.StopCam()
cv2.destroyAllWindows()

#####################################################################




# ########### Uncomment this code block for CAMERA CALIBRATION ############################
# 
# ImageSize = (544, 288)
# path="/home/pi/Desktop/chesspics544x288/"

# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objpr = np.zeros((9*7,3), np.float32)#7,9 number of chessboard corners  row and col
# objpr[:,:2] = (np.mgrid[0:7,0:9]*20).T.reshape(-1,2)# imultipled by 20 because length of chessboard square is 20mm

# # Arrays to store object points and image points from all the images.
# objpointsr = [] # 3d point in real world space
# imgpointsr = [] # 2d points in image plane.

# myLcorner=[]
# myRcorner=[]

# objpl =  np.zeros((9*7,3), np.float32)#np.zeros((9*7,3), np.float32)
# objpl[:,:2] = (np.mgrid[0:7,0:9]*20).T.reshape(-1,2)

# # Arrays to store object points and image points from all the images.
# objpointsl = [] # 3d point in real world space
# imgpointsl = [] # 2d points in image plane.

# counter = 0
# subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# print ("press any key to continue until \'DONE WITH CHESSNOARD CORNERS\' is shown\n")
# while counter<=50:
  
#     imgR=cv2.imread(path+str(counter)+'chessR.jpg')
    
#     grayr=cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)

#     imgL=cv2.imread(path+str(counter)+'chessL.jpg')
   
#     grayl=cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
    
#     try:
#         #finding corners
#         retval,cornersr= cv2.findChessboardCorners(grayr,(7,9),None)
#         cv2.cornerSubPix(grayr, cornersr,(11, 11),(-1,-1),subpix_criteria)
#         cv2.drawChessboardCorners(imgR,(7,9), cornersr, retval)
        

            
#         retval,cornersl= cv2.findChessboardCorners(grayl,(7,9),None)
#         cv2.cornerSubPix(grayl, cornersl,(11, 11),(-1,-1),subpix_criteria)
#         cv2.drawChessboardCorners(imgL,(7,9), cornersl, retval)


#         objpointsr.append(objpr)
#         imgpointsr.append(cornersr)
        
#         objpointsl.append(objpl)
#         imgpointsl.append(cornersl)
       
        
#         #show corners
       
#         #print str(counter)+'Right_image.jpg+Good'
#         #cv2.imwrite(str(counter)+'Right_image.jpg',imgR)
#         #cv2.imwrite(str(counter)+'imgRtemp.jpg',imgRtemp)
        
#         img=np.concatenate((imgL, imgR), axis=1)
#         cv2.imshow(str(counter)+'Left and Right image, chessboard corners drawn',img)
#         #print str(counter)+'Left_image.jpg+Good'
#         #cv2.imwrite(str(counter)+'Left_image.jpg',imgL)
#         #cv2.imwrite(str(counter)+'imgLtemp.jpg',imgLtemp)
        
        
        
#         counter=counter+1
        
#         c=cv2.waitKey(1)&0xFF
#         #if ord('q') == c:
#         cv2.destroyAllWindows()
#         print ("Next")

#     except:
#         counter=counter+1
#         print ('Not saved')
                

        
# print ("\n\nDONE WITH CHESSNOARD CORNERS\n\n")
# cv2.destroyAllWindows()


# print ("\n\nProcessing........, Please wait")
# N_OK = len(objpointsr)
# mtxr = np.zeros((3, 3))
# distr = np.zeros((4, 1))
# rvecsr = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
# tvecsr = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

# N_OK = len(objpointsl)
# mtxl = np.zeros((3, 3))
# distl = np.zeros((4, 1))
# rvecsl = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
# tvecsl = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

# #Camera matrix from here are good enough
# retr, mtxr, distr, rvecsr, tvecsr = cv2.calibrateCamera(objpointsr, imgpointsr, ImageSize,mtxr,distr,rvecsr, tvecsr)
                                                        
# retl, mtxl, distl, rvecsl, tvecsl = cv2.calibrateCamera(objpointsl, imgpointsl, ImageSize,mtxl,distl,rvecsl, tvecsl)



# mean_error = 0
# for i in range(len(objpointsr)):
#     imgpoints2r, _ = cv2.projectPoints(objpointsr[i], rvecsr[i], tvecsr[i], mtxr, distr)
#     error = cv2.norm(imgpointsr[i], imgpoints2r, cv2.NORM_L2)/len(imgpoints2r)
#     mean_error += error
# print( "total error: {}".format(mean_error/len(objpointsr)) )



# retval, cMatrixl, distCl, cMatrixr, distCr, R, T, E, F=cv2.stereoCalibrate(objpointsr,
#                                                                                      imgpointsl,
#                                                                                      imgpointsr,
#                                                                                      mtxl,distl,mtxr,distr,ImageSize,
#                                                                                      criteria=(cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5),
#                                                                                      flags=cv2.CALIB_FIX_INTRINSIC)


# Rl, Rr, Pl, Pr, Q, roil, roir= cv2.stereoRectify(cMatrixl, distCl, cMatrixr, distCr,
#                                                 ImageSize,R,T,None, None, None, None, None,flags=cv2.CALIB_ZERO_DISPARITY,alpha=-1)


# print ("\nRepro error R:")
# print (retr)
# print ("\nRepro error SR:")
# print (retval)
# print ("\n\n\n**************camera intrinsic and extrinsic properties ************")
# print ("\nRight camera matrix:")
# print (cMatrixr) 
# print ("\nLeft camera matrix:")
# print (cMatrixl)
# print ("\nRight camera distortion:")
# print (distCr)
# print ("\nLeft camera distortion:")
# print (distCl)
# print ("\nRight camera Projection matrix:")
# print (Pr)
# print ("\nLeft camera Projection matrix:")
# print (Pl)
# print ("\nRotation between cameras:")
# print (R)
# print ("\nTranslation between cameras:")
# print (T)
# print ("\nCalc. Essential Matrix R*S:")
# print (E)
# print ("\nCalc. Fundamental Matrix (M^-1)^T*E*(M^-1):")
# print (F)
# print ("\nReprojection matrix Q:")
# print (Q)


###############################################################################################

