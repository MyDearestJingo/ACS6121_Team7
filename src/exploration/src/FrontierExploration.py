#!/usr/bin/env python3
# Node for the frontier based exploration strategy

import rospy
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist, Point, Vector3, Transform
from tf2_msgs.msg import TFMessage
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import OccupancyGrid
# Helper functions for frontiers
import Helpers.FrontierHelpers as FH

'''
    Algorithm:
    ---
    We send move_base goals and recieve map data.

    When the map server updates the map:
        regenerate frontiers
        choose the best frontier to explore:
            choose closest frontier, which is also bigger than a threshold size (?)
        move_base to that frontier's centroid
    ---
    Every update of the map, we forget the previous goal and compute a new one because the previous goal is now obsolete.
    This prevents redundant movements in the path plan thus saving time.
    We choose nearest frontier as that prevents the robot from choosing an undesirable frontier (like a frontier from the
    other end of the map).

    The above algorithm does not necessarily explore all 9 zones but can map the full space
    Proposal:
        Record visited points
        Map points to the 9 zones
            The borderline points should be excluded as we want the robot to be completely inside a zone
        Force robot to visit any zones that have not been visited
'''

class ExplorationNode:

    # callback function that listens to map node of the slam package
    def MapUpdateListener(self, MapData :OccupancyGrid):
        self.Map = MapData
        print(f"Recieved a map update. Computing new frontiers")
        bFoundFrontier, Frontiers = self.FindFrontierToExplore(self.CurrentPose)
        if(bFoundFrontier):
            print(f"Moving to: {Frontiers[0].CentroidX}, {Frontiers[0].CentroidY} from: {self.CurrentPose}")
            self.SetGoal(Frontiers[0].CentroidX, Frontiers[0].CentroidY)
        else:
            print(f"No new frontier found.")

    def PoseListener(self, Transforms :TFMessage):
        for Transform in Transforms.transforms:
            if Transform.header.frame_id == "odom":
                self.CurrentPose = Transform.transform.translation
        
    def __init__(self):
        self.NodeName = "ExplorerAS"
        rospy.init_node(self.NodeName, anonymous=True)
        self.rate = rospy.Rate(10) # hz
        
        # setup move base client
        self.MoveBaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.MoveBaseClient.wait_for_server()

        # setup publishers and subscribers:
        self.MarkerPub = rospy.Publisher("visualization_marker", Marker)
        self.MapSub = rospy.Subscriber("map", OccupancyGrid, self.MapUpdateListener)
        self.PoseSub = rospy.Subscriber("tf", TFMessage, self.PoseListener)

        self.CurrentPose = Point()

        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {self.NodeName} node has been initialised...")

    # finds frontiers and returns the best frontier to explore currently
    def FindFrontierToExplore(self, Pose :Point):

        # create empty frontier list
        FrontierList = []

        Width = self.Map.info.width
        Height = self.Map.info.height

        frontier_flag = [False for i in range(Width * Height)]
        visited_flag = [False for i in range(Width * Height)]
        BFS = [] # Breadth first search

        # find closest clear cell to start search
        PosX, PosY = FH.WorldToMap(Pose.x, Pose.y, self.Map);
        pos = int((PosY - 1) * self.Map.info.width + PosX)
        bCellFound, Clear = FH.FindNearestCell(pos, FH.FREE_SPACE, self.Map)
        if (bCellFound):
            BFS.append(Clear)
        else:
            BFS.append(pos)
            rospy.WARN("Could not find nearby clear cell to start search")
        
        visited_flag[BFS[0]] = True;

        while (len(BFS) > 0):
            Index = BFS.pop(0)

            # iterate over 4-connected neighbourhood
            for nbr in FH.nhood4(Index, self.Map):
            # add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                if (self.Map.data[nbr] <= self.Map.data[Index] and not visited_flag[nbr]):
                    visited_flag[nbr] = True
                    BFS.append(nbr)
                    # check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                elif (FH.IsNewFrontierPoint(nbr, self.Map, frontier_flag)):
                    frontier_flag[nbr] = True
                    NewFrontier = FH.BuildNewFrontier(nbr, pos, frontier_flag, self.Map)
                    if (NewFrontier.Size >= 10):
                        print(f"Centroid of new Frontier = {NewFrontier.CentroidX}, {NewFrontier.CentroidY}, size = {NewFrontier.Size}")
                        FrontierList.append(NewFrontier)

        # set costs of frontiers and sort based on cost
        for frontier in FrontierList:
            frontier.cost = FH.FrontierCost(frontier, self.Map)
        FrontierList.sort(key = lambda x: x.Cost)

        return len(FrontierList) > 0, FrontierList
        
    # cancels all previous goals to move_base and sets the new goal
    def SetGoal(self, PosX, PosY):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = PosX
        goal.target_pose.pose.position.y= PosY
        goal.target_pose.pose.orientation.w = 1.0

        self.MoveBaseClient.cancel_all_goals()
        self.MoveBaseClient.send_goal(goal)

        marker = Marker()
        marker.pose.position = goal.target_pose.pose.position
        self.MarkerPub.publish(marker)

    # handles shutdown procedure
    def shutdownhook(self):
        # stop the robot
        self.MoveBaseClient.cancel_all_goals()

if __name__ == '__main__':
    ExpNodeInstance = ExplorationNode()
    rospy.spin()