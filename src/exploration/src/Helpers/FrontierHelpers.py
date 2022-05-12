#Contains helper functions for the Frontier based exploration algorithm

import math
from numpy import Infinity
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid

FREE_SPACE = 0
NO_INFORMATION = -1

class Frontier:
    Points = []
    Size = 1
    CentroidX = 0
    CentroidY = 0
    MinDist = Infinity
    Cost = Infinity

def nhood4(Index, MapData :OccupancyGrid):
    # get 4-connected neighbourhood indexes, check for edge of map
    out = []

    Width = MapData.info.width
    Height = MapData.info.height

    if (Index > Width * Height - 1):
        rospy.WARN("Evaluating nhood for offmap point")
        return out

    if (Index % Width > 0):
        out.append(Index - 1)
    if (Index % Width < Width - 1):
        out.append(Index + 1)
    if (Index >= Width):
        out.append(Index - Width)
    if (Index < Width * (Height - 1)):
        out.append(Index + Width)
    
    return out;

def nhood8(Index, MapData :OccupancyGrid):
    # get 8-connected neighbourhood indexes, check for edge of map
    out = nhood4(Index, MapData);

    Width = MapData.info.width
    Height = MapData.info.height;

    if (Index > Width * Height - 1):
        return out;

    if (Index % Width > 0 and Index >= Width):
        out.append(Index - 1 - Width);
    if (Index % Width > 0 and Index < Width * (Height - 1)):
        out.append(Index - 1 + Width);
    if (Index % Width < Width - 1 and Index >= Width):
        out.append(Index + 1 - Width);
    if (Index % Width < Width - 1 and Index < Width * (Height - 1)):
        out.append(Index + 1 + Width);

    return out;

def IsNewFrontierPoint(Index, MapData :OccupancyGrid, FrontierFlags):
    # check that cell is unknown and not already marked as frontier
    if (MapData.data[Index] != NO_INFORMATION or FrontierFlags[Index]):
        return False

    # frontier cells should have at least one cell in 4-connected neighbourhood that is free
    for nbr in nhood4(Index, MapData):
        if (MapData.data[nbr] == FREE_SPACE):
            return True

    return False

def FrontierCost(frontier :Frontier, Map :OccupancyGrid):
    return (frontier.MinDist * Map.info.resolution) - (frontier.Size * Map.info.resolution)

def BuildNewFrontier(StartCell, Reference, frontier_flag, Map: OccupancyGrid):
    # initialize frontier structure
    Output = Frontier()
    Output.CentroidX = 0
    Output.CentroidY = 0
    Output.size = 1
    Output.MinDist = Infinity

    # push initial gridcell onto queue
    BFS = []
    BFS.append(StartCell)

    while (len(BFS) > 0):
        Index = BFS.pop(0);

        # try adding cells in 8-connected neighborhood to frontier
        for nbr in nhood8(Index, Map):
        # check if neighbour is a potential frontier cell
            if (IsNewFrontierPoint(nbr, Map, frontier_flag)):
                # mark cell as frontier
                frontier_flag[nbr] = True
                Mx, My = IndexToCells(Index, Map)
                Wx, Wy = MapToWorld(Mx, My, Map)

                point = Point()
                point.x = Wx
                point.y = Wy
                Output.Points.append(point)

                # update frontier size
                Output.Size += 1

                # update centroid of frontier
                Output.CentroidX += Wx
                Output.CentroidY += Wy

                ReferenceX, ReferenceY = IndexToCells(Reference, Map)
                # determine frontier's distance from robot, going by closest gridcell to robot
                distance = math.sqrt(math.pow((ReferenceX - Mx), 2.0) + math.pow((ReferenceY - My), 2.0))
                if (distance < Output.MinDist):
                    Output.MinDist = distance

                # add to queue for breadth first search
                BFS.append(nbr);

    # average out frontier centroid
    Output.CentroidX /= Output.Size
    Output.CentroidY /= Output.Size
    return Output

def FindNearestCell(Start, Value, Map :OccupancyGrid):

    Width = Map.info.width
    Height = Map.info.height

    if (Start >= Width * Height):
        return False, 0

    # initialize breadth first search
    BFS = []
    visited_flag = [False for i in range(Width * Height)]

    # push initial cell
    BFS.append(Start)
    visited_flag[Start] = True

    # search for neighbouring cell matching value
    while (len(BFS) > 0):
        Index = BFS.pop(0)

        # return if cell of correct value is found
        if (Map.data[Index] == Value):
            return True, Index

        # iterate over all adjacent unvisited cells
        for nbr in nhood8(Index, Map):
            if (not visited_flag[nbr]):
                BFS.append(nbr)
                visited_flag[nbr] = True

    return False, 0

def WorldToMap(WorldX, WorldY, Map :OccupancyGrid):
    MapX = (WorldX - Map.info.origin.position.x) / Map.info.resolution
    MapY = (WorldY - Map.info.origin.position.y) / Map.info.resolution
    return MapX, MapY

def MapToWorld(MapX, MapY, Map :OccupancyGrid):
    WorldX = (MapX * Map.info.resolution) + Map.info.origin.position.x
    WorldY = (MapY * Map.info.resolution) + Map.info.origin.position.y
    return WorldX, WorldY

def IndexToCells(Index, MapData :OccupancyGrid):
    Mx = Index % MapData.info.width
    My = int(Index / MapData.info.width) + 1
    return Mx, My