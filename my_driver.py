#!/usr/bin/env python
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import glob
import rospy
from std_msgs.msg import Int32MultiArray
import time 
import os

class Line : 
    def __init__(self, start, end) : 
        self.start = start
        self.end = end
    def update(self, start, end) : 
        self.start = start 
        self.end = end
    def getMiddle(self) : 
        return (self.start + self.end) // 2 
    def __str__(self) : 
        if(self.start == 0 and self.end == 0) : 
            return "not setted"
        return "(%d, %d), min = %d" % (self.start, self.end, (self.start + self.end) // 2) 


width = 0
height = 0
maxLine = 0
maxAngle = 180
fig, ax = plt.subplots(2, 2)

lineSetted = False 
lines= [Line(0, 0) , Line(0, 0), Line(0, 0)] 


def pub_motor(Angle, Speed) : 
    drive_info = [Angle, Speed] 
    drive_info = Int32MultiArray(data = drive_info) 
    pub.publish(drive_info) 

def init() : 
    global cap, width, height, fig, ax, lineSetted, maxLine
    cap = cv2.VideoCapture('/home/hyorim/catkin_ws/src/xycar_simul/src/track-s.mkv')
    ret, frame = cap.read() 
    if not ret : 
        print("Error in opening track-s.mkv")
    height, width = frame.shape[:2]
    maxLine = width // 2 
    plt.ion()

def sobel(image) :
    blur = cv2.GaussianBlur(image, (5,5),0)
    sobel = cv2.Sobel(blur,cv2.CV_8U,1,0,3)
    return sobel

def region_of_interest(image): 
    region = np.array([[(0, height - 30), (width, height - 30), (width - 200, 250), (200, 250)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, region, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def get_histogram(image):
    cuttedImage = image[height//10:, :]
    ret, image_bin = cv2.threshold(cuttedImage, 125, 255, cv2.THRESH_BINARY)
    hist = np.sum(image_bin, axis = 0)
    return hist

def getLine(histogram) : 
    global lineSetted, lines
    start = []
    end = [] 
    thres = 100
    prev = histogram[0] 
    for i in range(1, len(histogram)) : 
        now = histogram[i]
        if prev < thres and now > thres : 
            start.append(i) 
        if prev > thres and now < thres : 
            end.append(i)
        prev = now 
    
    if len(start) != len(end) : 
        return
    if lineSetted == False and len(start) == 3 : 
        lineSetted = True
        print("At First, Lines are setted!!!")
        for i in range(3) : 
            lines[i].update(start[i], end[i]) 
        return 


    #do bipartite matching!!!
    if lineSetted == True and len(start) == 1 : 
        minIndex = 0 
        mindiff = 10000
        for i in range(3) : 
            diff = abs(lines[i].start - start[0]) + abs(lines[i].end - end[0]) 
            if( diff < mindiff) : 
                mindiff = diff 
                minIndex = i 
        lines[minIndex].update(start[0], end[0])
        if 0 != minIndex : 
            lines[0].start = 0
            lines[0].end = 0
        if 2 != minIndex : 
            lines[2].start = width
            lines[2].end = width
        return 

    if lineSetted == True and len(start) == 2 : 
        minIndex = set()
        mindiff = 10000 
        for i in range(3) : 
            dif = 0
            idx = 0
            for j in set(range(3)) - set([i]) : 
                dif = abs(lines[j].start - start[idx]) + abs(lines[j].end - end[idx])
                idx = idx + 1 
            if dif < mindiff : 
                mindiff = dif 
                minIndex = set(range(3)) - set([i])
        lines[list(minIndex)[0]].update(start[0], end[0])
        lines[list(minIndex)[1]].update(start[1], end[1]) 
        
        if not 0 in minIndex : 
            lines[0].start = 0
            lines[0].end = 0
        if not 2 in minIndex : 
            lines[2].start = width
            lines[2].end = width
        return 

    if lineSetted == True and len(start) == 3 : 
        for i in range(3) : 
            lines[i].update(start[i], end[i]) 
        return 

"""
def draw3Lines(frame) : 
    global lines, lineSetted 
    if lineSetted :
        cv2.line(frame, (lines[0].getMiddle(), height), (lines[0].getMiddle(), height - 20), (255, 0, 0), 10)
        cv2.line(frame, (lines[1].getMiddle(), height), (lines[1].getMiddle(), height - 20), (0, 255, 0), 10)
        cv2.line(frame, (lines[2].getMiddle(), height), (lines[2].getMiddle(), height - 20), (0, 0, 255), 10)
    return frame
"""

if __name__ == '__main__' :
    global pub, maxLine, maxAngle, lineSetted, lines
    rospy.init_node('my_driver')
    pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(30)
    speed = 20
    angle = 0

    init()
    while (cap.isOpened) :
        ret, frame = cap.read()

        if ret:
            grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sobeled_image = sobel(grayframe)
            cropped_image = region_of_interest(sobeled_image)
            hist = get_histogram(cropped_image)
            lines[0].update(0,0)
            lines[2].update(width,width)
            getLine(hist)
            if lineSetted :
                midLine = (lines[1].start + lines[1].end )//2
                diffLine = midLine - maxLine
                if(diffLine < -80) : 
                    angle = -50
                if(diffLine > 80) : 
                    angle  = 50  
                pub_motor(angle, speed)
                rate.sleep()
			
            #draw3Lines(frame) 
            cv2.imshow("gray", frame) 
            k = cv2.waitKey(1)
            if k == 27 : 
                break
        else : 
            break
    cap.release() 
    cv2.destroyAllWindows()
