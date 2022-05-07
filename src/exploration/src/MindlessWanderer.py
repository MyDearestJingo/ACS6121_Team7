#!/usr/bin/env python3
# Node for the frontier based exploration strategy

import rospy
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import OccupancyGrid

MAP_WIDTH = 3
MAP_HEIGHT = 3

'''
    Algorithm:
    ---
    We send move_base goals and wait for goal to be reached.

    The map is split into 9 grids and the size of the map is known.
    The robot is made to move to each of these 9 zones. (8 actually as it starts from one of the zones anyway.)

    We define the size of the map. Then split it onto 3 rows and 3 columns.
    The robot is told to a point in the grid such that the entire robot is inside the grid.
    If it cannot go to the centre of the grid, find closest reachable point that still lets the robot move into the grid completely.
    ---
'''

class ExplorationNode:

    # callback function that listens to map node of the slam package
    def MapUpdateListener(self, MapData :OccupancyGrid):
        self.Map = MapData
        print(f"Recieved a map update.")            

    def __init__(self):
        self.NodeName = "ExplorerAS"
        rospy.init_node(self.NodeName, anonymous=True)
        self.rate = rospy.Rate(10) # hz
        
        # setup move base client
        self.MoveBaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.MoveBaseClient.wait_for_server()

        # setup publishers and subscribers:
        self.MapSub = rospy.Subscriber("map", OccupancyGrid, self.MapUpdateListener)

        self.CurrentPose = Point()
        self.Goals = [False for i in range(9)]
        
        # We start at one of our goals
        self.OnGoalReached(actionlib.GoalStatus.SUCCEEDED, MoveBaseResult())

        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {self.NodeName} node has been initialised...")

    def MoveBaseFeedbackListener(self, Feedback :MoveBaseFeedback):
        self.CurrentPose = Feedback.base_position.pose.position

    def OnGoalReached(self, GoalStatus :int, GoalResult :MoveBaseResult):
        if(GoalStatus == actionlib.GoalStatus.SUCCEEDED):
            print(f"Goal reached. {self.FindGridNumber(self.CurrentPose)}")
            self.Goals[self.FindGridNumber(self.CurrentPose)] = 1
        else:
            print(f"Goal failed. {GoalStatus}, {self.FindGridNumber(self.CurrentPose)}")
            self.Goals[self.FindGridNumber(self.CurrentPose)] = -1

        for i in range(len(self.Goals)):
            if self.Goals[i] == 0:
                print(f"Moving to goal index {i}")
                self.SetGoalGrid(i)
                return
        print(f"No furthur goals.")

    def SetGoalGrid(self, GridIndex :int):
        x = GridIndex % 3
        y = int(GridIndex / 3)
        x-=1
        y-=1
        x = x * 1.3
        y = y * 1.3
        self.SetGoal(x,y)

    # cancels all previous goals to move_base and sets the new goal
    def SetGoal(self, PosX, PosY):
        print(f"Attempting to move to {PosX}, {PosY} from {self.CurrentPose}.")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = PosX
        goal.target_pose.pose.position.y= PosY
        goal.target_pose.pose.orientation.w = 1.0

        self.MoveBaseClient.cancel_all_goals()
        self.MoveBaseClient.send_goal(goal, done_cb=self.OnGoalReached, feedback_cb=self.MoveBaseFeedbackListener)

    def FindGridNumber(self, Location :Point):
        x = self.GetGridIndexFromDist(Location.x)
        y = self.GetGridIndexFromDist(Location.y)
        return (y * 3) + x

    def GetGridIndexFromDist(self, Dist :int):
        if(Dist < - 0.5):
            return 0
        elif Dist < 0.5:
            return 1
        else:
            return 2

    # handles shutdown procedure
    def shutdownhook(self):
        # stop the robot
        self.MoveBaseClient.cancel_all_goals()

if __name__ == '__main__':
    ExpNodeInstance = ExplorationNode()
    rospy.spin()