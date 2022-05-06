'''
Description: Provide functions for updating goal online
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-05-05 15:14:58
LastEditTime: 2022-05-06 12:28:07
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''

from math import atan2, pi, sqrt
import numpy as np


## Discription: update the goal oritentation dynamically according to current
#   position of robot and goal position for avoiding orientation adjusting when
#   the robot has arrived goal posttion.
## Params:
# - goalPos: a list containing the coordinate of goal position
# - currPos: a list containing the coordinate of current position
## Returns:
# - yaw: Euler yaw angle whose range is [-pi, pi]. yaw will less than 0 if the 
#   direction vector can be got by rotating Y axis anti-clockwise, vice versa.
## To-Do:
# [ ]. Figure out the relationship between quarternion and Euler yaw angle
# [ ]. Make sure that the yaw is updated right visually
def updateOrientation(goalPos, currPos):
    yaw = atan2(goalPos[0]-currPos[0], goalPos[1]-currPos[1]) - atan2(0,1)
    return yaw


## Discription: this function updates the goal position for preventing previous
#   desired position is unreachable
## Params:
# - goalPos: coordinate of desired position (meter in world coord system)
# - currPos: coordinate of current position (meter in world coord system)
# - costmap: local cost matrix in a format as np.ndarray in np.uint8
# - boxsize: size of searching box (cell)
# - res: resolution of costmap (m/cell)
## Returns:
# - a list contains the coordinate of new goal position in world system (m)
## To-Do:
# [ ]. More visualization tests are needed. 
# [ ]. Remove all debugging-purpose 'print' lines after testing fully
def updateGoal(goalPos, currPos, costmap, boxsize, res):

    h = costmap.shape[0]
    w = costmap.shape[1]
    
    rot = np.matrix([[0.0, -1.0, 0.0], 
                     [1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0]])
    xOm = currPos[0] - h*res/2      # origin of costmap in world coord sys
    yOm = currPos[1] + w*res/2      # origin of costmap in world coord sys
    print(f"cmap origin (m, world): ({xOm}, {yOm})")
    trans = np.matrix([[1.0, 0.0, -xOm],
                       [0.0, 1.0, -yOm],
                       [0.0, 0.0, 1.0]])
    homo = np.dot(rot, trans)       # homogenous matrix
    print(f"homo: \n{homo}")
    origin = np.array([[0,0,1]]).T
    print(f"world origin (m, cmap): {np.dot(homo, origin).T}")

    # coord of goal in costmap coord sys (cell)
    goalCmap = np.dot(homo, np.array([goalPos+[1]]).T) / \
        np.array([[res, res ,1]]).T
    # goalCmap -= np.array([1,1,0]).reshape((3,1))
    print(f"goal pos (cell, cmap): {goalCmap.T}")

    # left-top corner of searching box (cell, costmap coord sys)
    xbox = int(goalCmap[0,0]-boxsize/2)
    ybox = int(goalCmap[1,0]-boxsize/2)
    print(f"left-top of box (cell, cmap): ({xbox}, {ybox})")

    # check if the searching box is in costmap wholly
    if (xbox < 0 or xbox > h-boxsize-1 or \
        ybox < 0 or ybox > w-boxsize-1):
        return goalPos

    distPenalty = genDistPenalty(boxsize)
    zonecost = distPenalty + costmap[xbox:xbox+boxsize, ybox:ybox+boxsize]
    idx = zonecost.argmin()

    # coord of new goal (cell, costmap coord sys)
    goalNew = np.array([[int(idx/boxsize) + xbox, idx%boxsize + ybox ,1]]).T
    print(f"new goal pos (cell, cmap): ({goalNew.T}")

    # convert coord of new goal from costmap sys to world sys
    trans = np.matrix([[1.0, 0.0, xOm],
                       [0.0, 1.0, yOm],
                       [0.0, 0.0, 1.0]])
    homoReverse = np.dot(trans,rot.T)   # reverse homogenous matrix
    print(f"homeRevers: \n{homoReverse}")
    goalNew = (goalNew+0.5)*res         # locate the centre of grid in costmap
    goalNew[2,0] = 1
    print(f"new goal pos (m, cmap): ({goalNew.T})")
    # get the coord of new goal in world sys
    goalNew = np.dot(homoReverse, goalNew)  

    return [goalNew[0,0], goalNew[1,0]]


## Discription: to generate an distance penalty square matrix which added to an
#   area of costmap for selecting a reachable position closing to desired posi-
#   tion as possible
## Params:
# - boxsize: size of penalty matrix
# - w: weight, a hyper-parameter for controlling how strong this penalty is
## Returns:
# - a distance penalty square matrix
def genDistPenalty(boxsize, w=1):
    distPenalty = np.zeros((boxsize, boxsize), dtype=np.float32)
    for x in range(0, boxsize):
        for y in range(0, boxsize):
            distPenalty[x,y] = w * sqrt((x-(boxsize-1)/2)**2 + 
                (y-(boxsize-1)/2)**2)
    return distPenalty


if __name__ == "__main__":
    # goal = [1, 1]
    # curr = [0, 0]
    # print(updateOrientation(goal, curr))
    # d = genDistPenalty(3)
    # a = np.zeros((5,5))
    # print(a[:3, :3]+d)
    costmap = np.zeros((12,12))
    res = 0.5
    goal = [1.5, 1]
    curr = [0, 1]
    size = 5
    print(updateGoal(goal, curr, costmap, size, res))