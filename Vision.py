import time
import numpy as np
from GameElements import Disk
import cv2

class Vision(object): 

    

    def getDiskPosition(self, frame, disk):
        threshhold = 500

        position = (None, None)
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV, disk.lowHsv, disk.highHsv)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]  
    
        if len(contour_sizes) > 0:
            biggest_contour_size, biggest_contour = max(contour_sizes, key=lambda x: x[0])

            if biggest_contour_size > threshhold:

                x,y,w,h = cv2.boundingRect(biggest_contour)  
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(frame, disk.name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (36,255,12), 2)

                M = cv2.moments(biggest_contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                

                
                position = (cX, cY)
        
        
        return position


    def getTowerPosition(self, frame, tower):
        
        position = (None, None)

        arucoDict = cv2.aruco.Dictionary_get(tower.dictionary)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

        try:
            for (markerCorner, markerID) in zip(corners, ids):
                if markerID == tower.id:
                    x = 0
                    y = 0
                    for i in range(4):
                        x += markerCorner[0][i][0]
                        y += markerCorner[0][i][1]
                    
                    x /= 4
                    y /= 4


                    cv2.putText(frame, tower.name + " (" + str(x) + ", " + str(y) + ")", (int(x), int(y)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (36,255,12), 2)
                    
                    
                    position = (x, y)
        except: 
            position = (None, None)

        return position

    def getInitialState(self, frame, disks, towers):
        distanceThreshhold = 50

        state = []
        for tower in towers:
            diskList = []
            towerPosition = self.getTowerPosition(frame, tower)
            if towerPosition[0] == None:
                continue
            for disk in disks:
                diskPosition = self.getDiskPosition(frame, disk)
                if diskPosition[0] == None:
                    continue
                if(abs(diskPosition[0]-towerPosition[0])<=distanceThreshhold):
                    diskList.append(disk)
            
            state.append(diskList)
        
        for z in range(len(state)):
            for i in range(len(state[z])):
                min_index = i

                for j in range(i+1, len(state[z])):
                    if(self.getDiskPosition(frame, state[z][i])[1]<self.getDiskPosition(frame, state[z][j])[1]):
                        min_index = j
                

                (state[z][i], state[z][min_index]) = (state[z][min_index], state[z][i])

        return state