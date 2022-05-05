'''
Description: Saving OGM map
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-05-04 16:48:58
LastEditTime: 2022-05-05 01:55:15
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''

import rospy
import subprocess
import os

def saveMap(dir, mapname):
    if not os.path.exists(dir):
        os.makedirs(dir)

    subprocess.run(["rosrun", "map_server", 
        "map_saver", "-f", 
        dir+mapname])


if __name__ == "__main__":
    dir = "/home/student/catkin_ws/src/ACS6121_Team7/maps/"
    mapname = "map"
    saveMap(dir, mapname)