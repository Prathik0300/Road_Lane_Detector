import cv2 as cv
import numpy as np
from FindRoadBorder import DetectLine as dl
import FindRoadBorder as fbr
import math

def SteerControl(x,y):
    angle_rad = math.atan(x/y)
    angle_deg = int(angle_rad * 180/math.pi)
    steerAngle = angle_deg+90
    return steerAngle


video = cv.VideoCapture("v1.mp4")

while cv.waitKey(1)==-1:
    ret,frame = video.read()
    LaneCoord = dl(frame)
    print(LaneCoord)
    height,width,_ = frame.shape
    left,right = LaneCoord[0][0],LaneCoord[1][0]
    frame = LaneCoord[2]
    # left_history,right_history = LaneCoord[2],LaneCoord[3]
    if fbr.left_history_flag:
        right_x_min,right_y_min = right[0],right[1]
        right_x_max,right_y_max = right[2],right[3]
        x_offset_start = right_x_max - right_x_min
        y_offset = int(height/2)
        steerAngle = SteerControl(x_offset_start,y_offset)
        steerRad = (steerAngle * 180/math.pi)
        x_offset_end = (right_x_min-height / 2 / math.tan(steerRad))
        cv.line(frame,(width/2,height),(x_offset_end,y_offset),(0,255,0),3)

    elif fbr.right_history_flag:
        left_x_min,left_y_min = left[0],left[1]
        left_x_max,left_y_max = left[2],left[3]
        x_offset_start = left_x_max - left_x_min
        y_offset = int(height/2)
        steerAngle = SteerControl(x_offset_start,y_offset)
        steerRad = (steerAngle * 180/math.pi)
        x_offset_end = (left_x_min-height / 2 / math.tan(steerRad))
        cv.line(frame,(width/2,height),(x_offset_end,y_offset),(0,255,0),3)
    else:
        left_x_min,left_y_min = left[0],left[1]
        right_x_min,right_y_min = right[0],right[1]
        left_x_max,left_y_max = left[2],left[3]
        right_x_max,right_y_max = right[2],right[3]
        width = right_x_min - left_x_min
        x_offset_start = int((right_x_min+left_x_min)/2)
        x_offset_end = int((right_x_max+left_x_max)/2)
        y_offset = int(height*6/8)
        cv.line(frame,(x_offset_start,height),(x_offset_end,y_offset),(0,255,0),3)
    cv.imshow("frame",frame)
video.release()
cv.destroyAllWindows()

