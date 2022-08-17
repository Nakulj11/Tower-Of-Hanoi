
from logging import captureWarnings
import time
import numpy as np
from GameElements import Disk
from GameElements import Tower
from Vision import Vision
import cv2
import os

from robot_controller import MoveGroupPythonInterface


top_t1 = [0.7324, -0.8476, 1.2450, -1.9356, -1.5114, 0.7473]
top_t3 = [0.8781, -1.0205, 1.5414, -2.0682, -1.5070, 0.8929]
top_t2 = [0.8030, -0.9451, 1.4182, -2.0156, -1.5091, 0.8179]

d1_t1 = [0.7203068733215332, -0.7149899762919922, 1.3243983427630823, -2.171155115167135, -1.497375790272848, 0.7992434501647949]
d2_t1 = [0.7203307151794434, -0.7404855054667969, 1.316965405141012, -2.138294836083883, -1.4973519484149378, 0.7992434501647949]
d3_t1 = [0.7173700332641602, -0.7774384778789063, 1.312967602406637, -2.097036977807516, -1.497399632130758, 0.7962632179260254]

d1_t2 = [0.7955574989318848, -0.8030703824809571, 1.5014269987689417, -2.265691419641012, -1.4967759291278284, 0.874506950378418]
d2_t2 = [0.7956056594848633, -0.8435257238200684, 1.490213696156637, -2.214076181451315, -1.496739689503805, 0.8745427131652832]

d1_t3 = [0.8758759498596191, -0.8788140577128907, 1.6598723570453089, -2.354396482507223, -1.4965842405902308, 0.9549274444580078]

#disks, towers, and color ranges
disks = [Disk("a", np.array([80,100,100]), np.array([90,255,255])), #Green
         Disk("b", np.array([90,100,100]), np.array([100,255,255])), #Light Blue 
         Disk("c", np.array([101,120,90]), np.array([130,255,255])) #Dark Blue 
         ]
         

towers = [Tower("t1", cv2.aruco.DICT_4X4_50, 0),
          Tower("t2", cv2.aruco.DICT_4X4_50, 1),
          Tower("t3", cv2.aruco.DICT_4X4_50, 2)
        ]

cap = None
vision = None

def main():

    #video captures
    global cap
    cap = cv2.VideoCapture(2)

    time.sleep(5.0)

    global vision
    vision = Vision()



    for i in range(75):
        ret, frame = cap.read()
    
    ret, frame = cap.read()
    frame = frame[230:, 230:]

    while True:
        
        ret, frame = cap.read()        

        frame = frame[230:, 230:]
        for disk in disks:
            vision.getDiskPosition(frame, disk)
        
        for tower in towers:
            vision.getTowerPosition(frame, tower)

        cv2.imshow("Frame", frame)


        key = cv2.waitKey(1)

        if key == ord("q"):
            break




def close_gripper():
    print("Close the gripper then press [Enter]")
    raw_input()
    return

def open_gripper():
    print("Open the gripper then press [Enter]")
    raw_input()
    return

def gotoTower(tower):
    return
    
def lowerTower(tower):
    return
    
def lowerTowerPlace(args):
    disk1 = args[0]
    disk2 = args[1]
    tower = args[2]
    return
    
def pickUp(args):
    print("pickUp")
    print(args)
    gotoTower(args[-1])
    lowerTower(args[-1])
    close_gripper()
    gotoTower(args[-1])

def pickUpFromStack(args):
    print("pickUpFromeStack")
    print(args)
    gotoTower(args[-1])
    lowerTowerPlace(args)
    close_gripper()
    gotoTower(args[-1])

def placeOnTower(args):
    print("placeOnTower")
    print(args)
    gotoTower(args[-1])
    open_gripper()

def placeOnStack(args):
    print("placeOnStack")
    print(args)
    gotoTower(args[-1])
    open_gripper()


def getDisk(name):
    for disk in disks:
        if disk.name == name:
            return disk

    return None

def getTower(name):
    for tower in towers:
        if tower.name == tower:
            return tower

    return None


def getPlan(domain, problem):
    os.system("pyperplan " + domain + " " + problem)
    with open(problem + ".soln", 'r') as file:
        lines = file.readlines()
    
    return lines



def setInitialState(state):
    initial = ""

    for i in range(len(state)):
        if(len(state[i])==0):
            initial += "(EMPTYTOWER " + towers[i].name + ") "
        else:
            for j in range(len(state[i])):
                if j==0:
                    initial += "(TOPOFTOWER " + state[i][j].name + " " + towers[i].name + ") "
                    if j == len(state[i])-1:
                        initial += "(BOTTOMOFTOWER " + state[i][j].name + " " + towers[i].name + ") "
                else:
                    initial += "(ONDISK " + state[i][j-1].name + " " + state[i][j].name + " " + towers[i].name + ") "
                    if j == len(state[i])-1:
                        initial += "(BOTTOMOFTOWER " + state[i][j].name + " " + towers[i].name + ") "
    
    with open("problemHanoiTemplate.pddl", 'r') as file:
        lines = file.readlines()

    lines.insert(8, initial)

    with open("problemHanoi.pddl", 'w') as file:
        file.writelines(lines)
                    
if __name__ == "__main__":
    main()
