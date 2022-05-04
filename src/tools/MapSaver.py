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