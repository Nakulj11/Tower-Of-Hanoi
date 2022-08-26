# Tower-Of-Hanoi

Implementation of High-Level Planning 

High-Level Planning is a concept in robotics that involves robots completing order-sensitive tasks. In order for robots to perform competently in such environments, methods like task planning, motion planning, and more are combined in a High-Level system.

This repository contains the code for implementing High-Level Planning in the URe5 manipulator with the task of solving the puzzle, Tower of Hanoi. The domain of the environment, including predicates and available actions (with their subsequent pre-conditions and post-conditions), as well as the initial state and final state are recorded using the Planning Domain Definition Language (PDDL). This information is passed through a symbolic planner (Pyperplan) that plans the order of actions to be taken to reach the desired final state

Included is a vision system with the Intel RealSense Camera that is able to detect the position of the disks and towers. Using this information, the initial state is able to be updated and the system can arrive at the desired final state regardless of the starting positions of the disks. The vision system is also used during the robot movement to determine the height of disks, so the manipulator knows how low to go to pick up and place the disks

The combination of the vision system, motion planning, and task planning allows the High-Level System to be able to solve the Tower of Hanoi in a robust manner

Here is a demo of how it looks: https://www.youtube.com/watch?v=cZzbaRInxHk&t=2s
