
import cv2
import numpy as np
import cv_bridge
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


#Function for segmenting the orange vest
#Inputs:
    #-image: RGB image from camera
    #-lower_limit: lower HSV coefficients obtained from GA
    #-upper_limit: upper HSV coefficients
#Outputs:
    #-result: segmented binary image processed
def colorSegmentation(image,lower_limit,upper_limit):
    image_hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    normalized_hsv = cv2.normalize(image_hsv, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    mask=cv2.inRange(normalized_hsv,lower_limit,upper_limit)
    #cv2.imshow('Original binary',mask)
    kernel=np.ones((5,5),np.uint8)
    erode=cv2.erode(mask,kernel)
    #cv2.imshow('Erosion1',erode)
    dilate=cv2.dilate(erode,kernel,iterations=4)
    #cv2.imshow('Dilate1',dilate)
    erode=cv2.erode(dilate,kernel)
    #cv2.imshow('Erosion2',erode)
    result=erode
    return result



#Function for obtaining the distance to the vest segmented in ColorSegmentation()
#Inputs:
    #-segmented_result: output from ColorSegmentation()
    #-depth_image: depth image from camera
#Outputs:
    #-distance_to_vest: distance in m to the detected vest
def getDepthDistance(segmented_result,depth_image,raw_depth):
    #Extract boundind boxes from the segmented result by looking at the contours
    contours=cv2.findContours(segmented_result,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=contours[0] if len(contours) == 2 else contours[1]
    #Comparing all the contours to find out the biggest one
    max=0
    x,y,h,w=0,0,0,0
    for idx,cntr in enumerate(contours):
        xt,yt,wt,ht=cv2.boundingRect(cntr)
        xt=int(xt)
        yt=int(yt)
        wt=int(wt)
        ht=int(ht)
        size=wt*ht
        if size>max:
            max=size
            x,y,w,h=xt,yt,wt,ht
    
    cv2.rectangle(segmented_result, (x, y), (x+w, y+h), (255, 0, 0), 2)
    cv2.imshow('Segmentation', segmented_result)
    
    if w*h > 4000: #If the BB area is > 4000 it is considered valid
        x1=x
        x2=x+w
        y1=y
        y2=y+h

        #Check if the coordinates are lower or higher than dimensions
        xmax=depth_image.shape[1]#width
        ymax=depth_image.shape[0]#height
        xmin=0
        ymin=0
        if x1<xmin:
            x1=xmin
        if x1>xmax:
            x1=xmax
        if y1<ymin:
            y1=ymin
        if y1>ymax:
            y1=ymax
        if x2<xmin:
            x2=xmin
        if x2>xmax:
            x2=xmax
        if y2<ymin:
            y2=ymin
        if y2>ymax:
            y2=ymax
        
        
    
        # Use cv_bridge() to convert the ROS image to OpenCV format
        bridge = CvBridge()
        try:
            depth_image_array = bridge.imgmsg_to_cv2(raw_depth, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        
        
        depth_array=np.array(depth_image_array, dtype=np.float32)
        #Get the mean distance from the pixels of the bounding box
        mean_distance=0.0
        mean_counter=0.0
        for i in range(y1,y2):
            for j in range(x1,x2):
                mean_distance=mean_distance+(depth_array[int(i),int(j)])/1000
                mean_counter=mean_counter+1
        
        distance_to_vest=mean_distance/mean_counter
        return distance_to_vest
    else:
        return None