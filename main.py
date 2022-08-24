
from logging import captureWarnings
import time
import numpy as np
from GameElements import Disk
from GameElements import Tower
from Vision import Vision
import cv2
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

from robot_controller import MoveGroupPythonInterface


ur5e_arm = MoveGroupPythonInterface()


# joint states for every position
top_t1 = [0.7604665756225586, -0.701134518986084, 0.9556377569781702, -1.809092184106344, -1.5727341810809534, 5.453069686889648]
top_t3 = [0.8975424766540527, -0.8855036062053223, 1.3079608122455042, -1.9770733318724574, -1.570251766835348, 5.589624404907227]
top_t2 = [0.826603889465332, -0.8073514264873047, 1.1567061583148401, -1.9038769207396449, -1.5715106169330042, 5.518957614898682]

d1_t1 = [0.7596287727355957, -0.6340959829143067, 1.0258014837848108, -1.9463488064207972, -1.5727818647967737, 5.452148914337158]
d2_t1 = [0.7665920257568359, -0.6637891095927735, 1.0365989843951624, -1.927425046960348, -1.5726020971881312, 5.459064483642578]
d3_t1 = [0.7604665756225586, -0.6725123685649415, 0.9960525671588343, -1.8780952892699183, -1.5727461020099085, 5.452998161315918]

d1_t2 = [0.8266158103942871, -0.7413008970073243, 1.215790096913473, -2.0291296444334925, -1.5714986960040491, 5.518837928771973]
d2_t2 = [0.8304023742675781, -0.7639740270427247, 1.213093105946676, -2.00378479580068, -1.5713909308062952, 5.522709846496582]
d3_t2 = [0.8304266929626465, -0.7858198446086426, 1.1951525847064417, -1.9639092884459437, -1.5714510122882288, 5.522722244262695]

d1_t3 = [0.8974943161010742, -0.8219168943217774, 1.3572543303119105, -2.089963575402731, -1.570251766835348, 5.589576721191406]
d2_t3 = [0.9004526138305664, -0.8444345754436036, 1.3500712553607386, -2.060268064538473, -1.570179287587301, 5.59259033203125]
d3_t3 = [0.9004883766174316, -0.8684976857951661, 1.331806484852926, -2.017904897729391, -1.5702036062823694, 5.59259033203125]


#disks, towers, and color ranges
disks = [Disk("a", np.array([73,90,100]), np.array([90,255,255])), #Green
         Disk("b", np.array([90,90,100]), np.array([98,255,255])), #Light Blue 
         Disk("c", np.array([99,100,90]), np.array([130,256,256])) #Dark Blue 
         ]
         

towers = [Tower("t1", cv2.aruco.DICT_4X4_50, 0),
          Tower("t2", cv2.aruco.DICT_4X4_50, 1),
          Tower("t3", cv2.aruco.DICT_4X4_50, 2)
        ]

vision = None
vh = None

#vision handler, subscribes to color and depth image
class VisionHandler(object):

    def __init__(self):
        rospy.init_node('main', anonymous=True)
        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        
    def rgb_callback(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        
    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    def get_rgb_img(self):
        return np.copy(self.rgb_img)
        
    def get_depth_img(self):
        return np.copy(self.depth_img)

point = (0, 0)
    
    
# thread to display image frame and depth
def displayThread(vh):
    
    #frame = vh.get_rgb_img() 
    #cv2.imshow("Frame", frame)

    while True:
        
        #displays every disk and tower

        #ret, frame = cap.read()
        frame = vh.get_rgb_img() 
        #depth = vh.get_depth_img()
    
        #frame = frame[230:, 230:]
        #depth = depth[230:, 230:]

        #print(str(frame.shape) + " " + str(depth.shape))

        for disk in disks:
            vision.getDiskPosition(frame, disk)
        
        for tower in towers:
            vision.getTowerPosition(frame, tower)


        cv2.imshow("Frame", frame)
        #cv2.imshow("Depth", depth)

        key = cv2.waitKey(1)

        if key == ord("q"):
            # print(vision.convertToRealWorld([9, 98, 21]))
            break



def main():

    global vh
    vh = VisionHandler()
    
    rospy.sleep(1.0)

    global vision
    vision = Vision()


    #starts vision thread
    x = threading.Thread(target=displayThread, args=(vh,))
    x.start()
    
    
    #reads and writes inital state then runs planner
    setInitialState(vision.getInitialState(vh.get_rgb_img(), disks, towers))
    plan = getPlan("domainHanoi.pddl", "problemHanoi.pddl")
    #plan = []
    steps = {1: pickUp,
              2: pickUpFromStack,
              3: placeOnTower,
              4: placeOnStack}

    print("Press [Enter] to start")
    raw_input()

    #iterates through every step of the plan and runs corresponding method
    for step in plan:
        print("Press [Enter] to proceed")
        raw_input()
        print(step)
        step = step[1:-2]
        elements = step.split()

        steps[int(elements[0][-1])](elements[1:len(elements)])


    #while True:
        #pass
        #frame = vh.get_rgb_img()
        
        #for disk in disks:
            #vision.getDiskPosition(frame, disk)
        
        #for tower in towers:
            #vision.getTowerPosition(frame, tower)

        #cv2.imshow("Frame", frame)


        #key = cv2.waitKey(1)

        #if key == ord("q"):
            #break

#converts disk color coordinate to real world robot pov coordinates
def color_to_robot(pos):
    depth_pos = vision.convert_color_to_depth(pos)
    
    z = vh.get_depth_img()[int(depth_pos[1]), int(depth_pos[0])]
    
    robot_pos = vision.convert_depth_to_world([int(depth_pos[0]), int(depth_pos[1]), z])
    return robot_pos


#prompts close and open gripper
def close_gripper():
    print("Close the gripper then press [Enter]")
    raw_input()
    return

def open_gripper():
    print("Open the gripper then press [Enter]")
    raw_input()
    return

#goes to the top of tower specified in argument
def gotoTower(tower):
    if tower == 't1':
        ur5e_arm.goto_joint_state(top_t1)
    elif tower == 't2':
        ur5e_arm.goto_joint_state(top_t2)
    elif tower == 't3':
        ur5e_arm.goto_joint_state(top_t3)
    else:
        print('AAAAAAAAAAAAAAAAAAAAAAAAA')
    return
    
#lowers to the very bottom of a tower
def lowerTower(tower):
    if tower == 't1':
        ur5e_arm.goto_joint_state(d1_t1)
    elif tower == 't2':
        ur5e_arm.goto_joint_state(d1_t2)
    elif tower == 't3':
        ur5e_arm.goto_joint_state(d1_t3)
    else:
        print('AAAAAAAAAAAAAAAAAAAAAAAAA')
    return
    
#very sketchy
#lowers to specific part of tower based on state 
def lowerTowerPlace(args):
    disk1 = args[0]
    disk2 = args[1]
    tower = args[2]
    
    #gets state
    state = vision.getInitialState(vh.get_rgb_img(), disks, towers)
    
    disk2_position = -1
    
    """"
    #really sketch part to determine how low to go
    for i in range(len(state)):
        for j in range(len(state[i])):
            if state[i][j] == getDisk(disk2):
                if len(state[i])==3:
                    disk2_position = 1
                elif len(state[i])==1:
                    disk2_position = 0
                elif len(state[i])==2:
                    if j == 0:
                        disk2_position = 1
                    elif j == 1:
                        disk2_position = 0 
    """
    #determines how low to go when grabbing/placing disk
    for i in range(len(state)):
        tower = state[i]
        tower.reverse()
        for j in range(tower):
            if tower[j] == getDisk(disk2):
                disk2_position = j
    print(disk2_position)

    #lowers to specific height based on tower
    if tower == 't1' :
        if disk2_position == 0:
            ur5e_arm.goto_joint_state(d2_t1)
        elif disk2_position == 1:
          ur5e_arm.goto_joint_state(d3_t1)
    if tower == 't2' :
        if disk2_position == 0:
            ur5e_arm.goto_joint_state(d2_t2)
        elif disk2_position == 1:
          ur5e_arm.goto_joint_state(d3_t2)
    if tower == 't3':
        if disk2_position == 0:
            ur5e_arm.goto_joint_state(d2_t3)
        elif disk2_position == 1:
          ur5e_arm.goto_joint_state(d3_t3)
    else:
        print('AAAAAAAAAAAAAAAAAAAAAAAAA')
    return
    
#pick up [targetDisk, tower]
def pickUp(args):
    print("pickUp")
    print(args)
    gotoTower(args[-1])
    lowerTower(args[-1])
    close_gripper()
    gotoTower(args[-1])

#pick up from stack [targetDisk, diskBelowTarget, tower]
def pickUpFromStack(args):
    print("pickUpFromStack")
    print(args)
    gotoTower(args[-1])
    lowerTowerPlace(args)
    close_gripper()
    gotoTower(args[-1])


#place on tower [targetDisk, tower]
def placeOnTower(args):
    print("placeOnTower")
    print(args)
    gotoTower(args[-1])
    lowerTower(args[1])
    open_gripper()
    gotoTower(args[-1])

#place on tower [targetDisk, diskBelowTarget, tower]
def placeOnStack(args):
    print("placeOnStack")
    print(args)
    gotoTower(args[-1])
    lowerTowerPlace(args)
    open_gripper()
    gotoTower(args[-1])

#gets disk based on name
def getDisk(name):
    for disk in disks:
        if disk.name == name:
            return disk

    return None

#gets disk based on tower
def getTower(name):
    for tower in towers:
        if tower.name == name:
            return tower

    return None


#runs domain and plan with pyperplan
def getPlan(domain, problem):
    os.system("pyperplan " + domain + " " + problem)
    with open(problem + ".soln", 'r') as file:
        lines = file.readlines()
    
    return lines


#sets initial state to problemHanoi.pddl
def setInitialState(state):
    initial = ""

    for i in range(len(state)):
        #empty tower for a tower with 0 disks
        if(len(state[i])==0):
            initial += "(EMPTYTOWER " + towers[i].name + ") "
        else:
            
            for j in range(len(state[i])):
                #if it's the only disk, it's the top and bottom of the tower 
                if j==0:
                    initial += "(TOPOFTOWER " + state[i][j].name + " " + towers[i].name + ") "
                    if j == len(state[i])-1:
                        initial += "(BOTTOMOFTOWER " + state[i][j].name + " " + towers[i].name + ") "
                else:
                    #otherwise sets on disk and bottom
                    initial += "(ONDISK " + state[i][j-1].name + " " + state[i][j].name + " " + towers[i].name + ") "
                    if j == len(state[i])-1:
                        initial += "(BOTTOMOFTOWER " + state[i][j].name + " " + towers[i].name + ") "
    
    #inserts pddl into problemHanoi file
    with open("problemHanoiTemplate.pddl", 'r') as file:
        lines = file.readlines()

    lines.insert(8, initial)

    with open("problemHanoi.pddl", 'w') as file:
        file.writelines(lines)
                    
if __name__ == "__main__":
    main()

