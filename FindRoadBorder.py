import cv2 as cv
import numpy as np
import matplotlib.image as mpimg
import math
from numpy.core.numeric import Inf
from numpy.lib.type_check import mintypecode

left_history_flag = False
right_history_flag = False
lower_white = np.array([0,0,168])
upper_white = np.array([172,11,255])
left_border,right_border = [[0,0,0,0]],[[0,0,0,0]]
history_right = []
history_left = []

def CroppingImage(img,vertice):
    mask = np.zeros_like(img)
    cv.fillPoly(mask,vertice,(255,0,0))
    res = cv.bitwise_and(img,mask)
    return res

def DrawLine(img,lines):
    left_x = []
    left_y = []
    right_x = []
    right_y = []
    height,width,_ = img.shape
    bound = 1/3
    left_bound = width * (1-bound)
    right_bound = width*bound

    try:
        for line in lines:
            for x1,y1,x2,y2 in line:
                try:
                    slope = (y2-y1)/(x2-x1)
                except:
                    continue
                if math.fabs(slope)<0.5:
                    continue
                if slope<0:
                    if x1<left_bound and x2<left_bound:
                        left_x.extend([x1,x2])
                        left_y.extend([y1,y2])
                else:
                    if x1>right_bound and x2>right_bound:
                        right_x.extend([x1,x2])
                        right_y.extend([y1,y2])
    except:
        return None

    max_y = int(img.shape[0])             
    min_y = int(img.shape[0]*3/4)

    try:
        curve_left = np.poly1d(np.polyfit(
            left_y,
            left_x,
            deg=1
        ))
        left_side_min = int(curve_left(max_y))
        left_side_max = int(curve_left(min_y))
        left_border[0] = [left_side_min,max_y,left_side_max,min_y]
        cv.line(img,(left_side_min,max_y),(left_side_max,min_y),(0,0,255),6)
        history_left.append([left_side_min,max_y,left_side_max,min_y])
        left_history_flag = False
    except:
        left_history_flag=True
        print("left lane miss",left_history_flag)
        left_border[0] = [0,0,0,0]
        pass
        # if len(history_left)<1:
        #     pass
        # else:
        #     val = history_left[-1]
        #     x1,y1,x2,y2 = val[0],val[1],val[2],val[3]
        #     left_border[0] = [x1,y1,x2,y2]
        #     cv.line(img,(x1,y1),(x2,y2),(0,0,255),6)    

    try:
        curve_right = np.poly1d(np.polyfit(
        right_y,
        right_x,
        deg=1
     ))   
        right_side_min = int(curve_right(max_y))
        right_side_max = int(curve_right(min_y))
        right_border[0] = [right_side_min,max_y,right_side_max,min_y]
        cv.line(img,(right_side_min,max_y),(right_side_max,min_y),(0,0,255),6)
        history_right.append([right_side_min,max_y,right_side_max,min_y])
        right_history_flag=False
    except:
        right_history_flag=True
        print("right lane miss",right_history_flag)
        right_border[0] = [0,0,0,0]
        pass
        # if len(history_right)<1:
        #     pass
        # else:
        #     val = history_right[-1]
        #     x1,y1,x2,y2 = val[0],val[1],val[2],val[3]
        #     right_border[0] = [x1,y1,x2,y2]
        #     cv.line(img,(x1,y1),(x2,y2),(0,0,255),6)
        
    return (left_border,right_border,img,left_history_flag,right_history_flag)

def DetectLine(frame):
    height = frame.shape[0]
    width = frame.shape[1]
    for i in range(height//2):
        for j in range(width):
            frame[i][j]=0
    roi_vertices = [
        (10,height),
        (width/1.2,height/1.2),
        (width*3/4,height)
    ]
    frame1 = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(frame1,(3,3),3)
    edge = cv.Canny(blur,100,110)
    lines = cv.HoughLinesP(edge,2,np.pi/60,150,lines = np.array([]),minLineLength = 40,maxLineGap=25)
    BorderPoints = DrawLine(frame,lines)
    return BorderPoints

# def loop(v):
#     video = cv.VideoCapture("v2.mp4")
#     return video
# video = cv.VideoCapture("v2.mp4")
# while cv.waitKey(1)==-1:
#     ret,img = video.read()
#     if ret==0:
#         video = loop(video)
#         continue
#     res = DetectLine(img)
#     # cv.imshow("mask",v3)
#     # cv.imshow("edge",v1)
#     # cv.imshow("roi",v2)

#     if res is None:
#         cv.imshow("final",img)
#     else:
#         cv.imshow("final",res)
# video.release()
# cv.destroyAllWindows()
