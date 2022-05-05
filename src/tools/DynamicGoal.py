'''
Description: Saving OGM map
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-05-05 15:14:58
LastEditTime: 2022-05-05 15:14:58
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''

from math import atan2, pi

def updateOrientation(goalPos, currPos):
    # v = goalPos - currPos
    yaw = atan2(goalPos[0]-currPos[0], goalPos[1]-currPos[1]) - atan2(0,1)
    return yaw


if __name__ == "__main__":
    goal = [1, 1]
    curr = [0, 0]
    print(updateOrientation(goal, curr))