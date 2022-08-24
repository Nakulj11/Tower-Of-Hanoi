import time
import numpy as np
from GameElements import Disk
import cv2

class Vision(object): 

    def __init__(self):
        self.setAffineTransformation()
    
    #sad attempt at setting transformation from color fram pixel points to 
    #real world robot coordinates
    def setAffineTransformation(self):

        #source and destination points
        pts_src = np.array([[146, 230, 112], 
                            [387, 176, 120], 
                            [314, 164, 109], 
                            [348, 78, 145], 
                            [271, 168, 98], 
                            [416, 145, 136], 
                            [512, 143, 87], 
                            [361, 239, 174], 
                            [269, 299, 96], 
                            [502, 223, 105], 
                            [318, 233, 106], 
                            [307, 253, 78], 
                            [375, 186, 165], 
                            [392, 213, 129], 
                            [375, 283, 94], 
                            [462, 217, 136]
                            ]).astype(np.float32)
        
        pts_dst = np.array([[0.2280, 0.4314, 0.1469], 
                            [0.4098, 0.5202, 0.1536], 
                            [0.3494, 0.4476, 0.2014], 
                            [0.3792, 0.6062, 0.2264], 
                            [0.3048, 0.4664, 0.1659], 
                            [0.4620, 0.6235, 0.1456], 
                            [0.4770, 0.4292, 0.1967], 
                            [0.4108, 0.6876, 0.0072], 
                            [0.2947, 0.4470, 0.0736], 
                            [0.4847, 0.4315, 0.1165], 
                            [0.3477, 0.4789, 0.1006], 
                            [0.3354, 0.3711, 0.1627], 
                            [0.4179, 0.6584, 0.0870], 
                            [0.4253, 0.5422, 0.0831], 
                            [0.3852, 0.4237, 0.0873], 
                            [0.5174, 0.6118, 0.0506] 
                            ]).astype(np.float32)

        #source and destination points
        pts_src_color = np.array([[116, 214], 
                            [469, 136], 
                            [361, 109], 
                            [395, 5], 
                            [289, 132], 
                            [518, 97], 
                            [631, 72], 
                            [426, 229], 
                            [307, 321], 
                            [634, 267], 
                            [374, 227], 
                            [344, 232], 
                            [448, 149], 
                            [478, 208], 
                            [452, 287], 
                            [592, 200]
                            ]).astype(np.float32)
        
        pts_dst_depth = np.array([[146, 230], 
                            [387, 176], 
                            [314, 164], 
                            [348, 78], 
                            [271, 168], 
                            [416, 145], 
                            [512, 143], 
                            [361, 239], 
                            [269, 299], 
                            [502, 223], 
                            [318, 233], 
                            [307, 253], 
                            [375, 186], 
                            [392, 213], 
                            [375, 283], 
                            [462, 217]
                            ]).astype(np.float32)
                            
        ret, self.M, mask = cv2.estimateAffine3D(pts_src, pts_dst, cv2.RANSAC)
        print(self.M)
            
        self.M2, status = cv2.findHomography(pts_src_color, pts_dst_depth)
        print(self.M2)

    #tranforms any point based on affine transformation
    def convertToRealWorld(self, point):
        result = np.array((self.M * np.vstack((np.matrix(point).reshape(3, 1), 1)))[0:3, :].reshape(1, 3))[0]

        return result
        
    #converts an array of 2 pts from color frame pts to depth frame pts
    def convert_color_to_depth(self, point):
        result = np.array((self.M2 * np.vstack((np.matrix(point).reshape(2, 1), 1)))[0:3, :].reshape(1, 3))[0]

        return result
        
    #converts [x, y, depth] to [i, j, k] for the robot
    def convert_depth_to_world(self, point):
        result = np.array(( self.M * np.vstack((np.matrix(point).reshape(3, 1), 1)) )).reshape((3,))

        return result

    #returns disk x y and depth
    def getDiskPosition(self, frame, disk):
        threshhold = 50

        #creates HSV mask
        position = (None, None)
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(frameHSV, disk.lowHsv, disk.highHsv)
        #cv2.imshow("Frame",frameHSV)
        #cv2.waitKey(1)

        #finds the biggest contours
        img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]  
    
        if len(contour_sizes) > 0:
            biggest_contour_size, biggest_contour = max(contour_sizes, key=lambda x: x[0])

            if biggest_contour_size > threshhold:

                x,y,w,h = cv2.boundingRect(biggest_contour)  
                
                M = cv2.moments(biggest_contour)
                #center pixels 
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                
                position = (cX, cY)

                #draws rectangles
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(frame, disk.name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (36,255,12), 2)

        
        
        return position


    # returns tower x y and depth
    def getTowerPosition(self, frame, tower):
        
        position = (None, None)

        #gets aruco marker position
        arucoDict = cv2.aruco.Dictionary_get(tower.dictionary)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

        try:
            for (markerCorner, markerID) in zip(corners, ids):
                if markerID == tower.id:
                    #calculating center of the marker
                    x = 0
                    y = 0
                    for i in range(4):
                        x += markerCorner[0][i][0]
                        y += markerCorner[0][i][1]
                    
                    x /= 4
                    y /= 4

                    #putting text at tower position
                    cv2.putText(frame, tower.name, (int(x), int(y)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (36,255,12), 2)
                    
                    
                    position = (x, y)
        except: 
            position = (None, None)

        return position

    # returns [tower1_state, tower2_state, tower3_state]
    #towerX_state is a list of disks from top to bottom
    #example for normal configuration
    #[[a, b, c],[],[]]
    def getInitialState(self, frame, disks, towers):
        distanceThreshhold = 55

        state = []
        #goes through each tower
        for tower in towers:
            diskList = []
            towerPosition = self.getTowerPosition(frame, tower)
            print(tower.name + str(towerPosition))
            if towerPosition[0] == None:
                continue
            for disk in disks:
                #goes through and gets every disk position
                diskPosition = self.getDiskPosition(frame, disk)
                print(disk.name + str(diskPosition))
                if diskPosition[0] == None:
                    continue
                #checks if disk is within a certain threshold of tower x
                if(abs(diskPosition[0]-towerPosition[0])<=distanceThreshhold):
                    diskList.append(disk)
            
            state.append(diskList)
        
        #sorts disks from top to bottom based on disk position y
        for z in range(len(state)):
            for i in range(len(state[z])):
                min_index = i

                for j in range(i+1, len(state[z])):
                
                    
                    if(self.getDiskPosition(frame, state[z][i])[1]>self.getDiskPosition(frame, state[z][j])[1]):
                        print(state[z][i].name + " " + state[z][j].name)
                        print(str(self.getDiskPosition(frame, state[z][i])) + " " + str(self.getDiskPosition(frame, state[z][j])))
                        min_index = j
                
                temp = state[z][i]
                state[z][i] = state[z][min_index]
                state[z][min_index] = temp
                #(state[z][i], state[z][min_index]) = (state[z][min_index], state[z][i])
       
       
        for i in state:
            for j in i:
                print(j.name)
       
        return state


if __name__ == '__main__':
    v = Vision()
