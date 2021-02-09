import cv2 as cv
import numpy as np
from FindRoadBorder import DetectLine as dl
import math

def stabilization(curr_angle,new_angle,lane,max_dev_2l = 4,max_dev_1l = 1):
    max_deviation = None
    if lane ==2:
        max_deviation = max_dev_2l
    elif lane==1:
        max_deviation = max_dev_1l
    deviation = new_angle - curr_angle
    if max_deviation is not None and abs(deviation)>max_deviation:
        final_angle = int(curr_angle+deviation * max_deviation / abs(deviation))
    else:
        final_angle = new_angle
    return final_angle

def SteerControl(x,y,turnLeft = False):
    angle_rad = math.atan(x/y)
    angle_deg = int(angle_rad * 180/math.pi)
    steerAngle = angle_deg + 90
    return steerAngle

LaneLines = 2
old_steer_angle = 90
video = cv.VideoCapture("./Input Data/v1.mp4")

while cv.waitKey(1)==-1:
    ret,frame = video.read()
    LaneCoord = dl(frame)
    height,width,_ = frame.shape
    left,right = LaneCoord[0][0],LaneCoord[1][0]
    frame = LaneCoord[2]
    left_history_flag,right_history_flag = LaneCoord[3],LaneCoord[4]
    if left_history_flag==True:
        LaneLines=1
        right_x_min,right_y_min = right[0],right[1]
        right_x_max,right_y_max = right[2],right[3]
        x_offset_start = right_x_max - right_x_min
        y_offset = int(height * 8/9)
        steerAngle = SteerControl(x_offset_start,y_offset)
        required_steer_angle = stabilization(old_steer_angle,steerAngle,LaneLines)
        old_steer_angle = steerAngle
        steerRad = (required_steer_angle * 180/math.pi)
        val = math.tan(steerRad) 
        x_offset_end = int(right_x_min-height / 2 / val)
        cv.line(frame,(width//2,height),(x_offset_end,y_offset),(0,255,0),3)
    elif right_history_flag==True:
        LaneLines=1
        left_x_min,left_y_min = left[0],left[1]
        left_x_max,left_y_max = left[2],left[3]
        x_offset_start = left_x_max - left_x_min
        y_offset = int(height * 8/9)
        steerAngle = SteerControl(x_offset_start,y_offset,True)
        required_steer_angle = stabilization(old_steer_angle,steerAngle,LaneLines)
        old_steer_angle = steerAngle
        steerRad = (required_steer_angle * 180/math.pi)
        val = math.tan(steerRad) 
        x_offset_end = int(left_x_min-height / 2 / val)
        cv.line(frame,(width//2,height),(x_offset_end,y_offset),(0,255,0),3)
    else:
        LaneLines=2
        left_x_min,left_y_min = left[0],left[1]
        right_x_min,right_y_min = right[0],right[1]
        left_x_max,left_y_max = left[2],left[3]
        right_x_max,right_y_max = right[2],right[3]
        width = right_x_min - left_x_min
        x_offset_start = int((right_x_min+left_x_min)/2)
        x_offset_end = int((right_x_max+left_x_max)/2)
        y_offset = int(height*6/8)
        steerAngle = SteerControl(x_offset_start,y_offset)
        required_steer_angle = stabilization(old_steer_angle,steerAngle,LaneLines)
        cv.line(frame,(x_offset_start,height),(x_offset_end,y_offset),(0,255,0),3)
    if required_steer_angle>90:
        print("turning right ",required_steer_angle)
    elif required_steer_angle<90:
        print("turning left ",required_steer_angle)
    else:
        print("straight ",required_steer_angle)
    cv.imshow("frame",frame)
video.release()
cv.destroyAllWindows()

