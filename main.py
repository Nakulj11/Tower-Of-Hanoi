
from logging import captureWarnings
import time
import numpy as np
from GameElements import Disk
from GameElements import Tower
from Vision import Vision
import cv2
import os


#disks, towers, and color ranges
disks = [Disk("a", np.array([60,120,120]), np.array([85,255,255])), #Green
         Disk("b", np.array([160,25,76]), np.array([170,155,255])) #Purple
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
    cap = cv2.VideoCapture(0)

    time.sleep(2.0)

    global vision
    vision = Vision()



    # ret, frame = cap.read()

    # setInitialState(vision.getInitialState(frame, disks, towers))
    # plan = getPlan("domainHanoi.pddl", "problemHanoi.pddl")

    # steps = {1: pickUp,
    #          2: pickUpFromStack,
    #          3: placeOnTower,
    #          4: placeOnStack}

    # for step in plan:
    #     step = step[1:-2]
    #     elements = step.split()

    #     steps[int(elements[0][-1])](elements[1:len(elements)])


    while True:
        
        ret, frame = cap.read()        

        for disk in disks:
            vision.getDiskPosition(frame, disk)
        
        for tower in towers:
            vision.getTowerPosition(frame, tower)

        cv2.imshow("Frame", frame)


        key = cv2.waitKey(1)

        if key == ord("q"):
            break




def pickUp(args):
    pass

def pickUpFromStack(args):
    pass

def placeOnTower(args):
    pass

def placeOnStack(args):
    pass


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