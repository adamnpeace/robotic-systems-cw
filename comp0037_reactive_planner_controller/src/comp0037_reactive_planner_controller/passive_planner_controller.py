# This class handles the passive planner and controller. The passive planner controller simply
# drives the robots from start to goal and assumes that the robot can always get there. It cannot
# handle the case, for example, that an unexpected obstable appears.

import rospy

import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *

class PassivePlannerController(PlannerControllerBase):
    seen = []

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()

    def mapUpdateCallback(self, mapUpdateMessage):
        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return
    
    def driveToGoal(self, goal):
        # Exit if we need to
        if rospy.is_shutdown() is True:
            return False

        # Get the coal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Plan a path using the current occupancy grid
        pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)

        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                          goalCellCoords[0], goalCellCoords[1])
            return False
            
        # Extract the path
        self.currentPlannedPath = self.planner.extractPathToGoal()

        # Drive along the path the goal
        goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, goal.theta, self.planner.getPlannerDrawer())

        numUnseen = 0
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                # print("self x y", x, y, self.occupancyGrid.getCell(x, y))
                if self.occupancyGrid.getCell(x, y) == 0.5:
                    numUnseen += 1
        totalCells = self.occupancyGrid.getWidthInCells() * self.occupancyGrid.getHeightInCells()
        coverage = 100 - ((1000*float(numUnseen)/float(totalCells))//1)/10
        print "Number of cells unseen:", numUnseen, "out of", totalCells, "giving", coverage, "% coverage."

        rospy.logerr('goalReached=%d', goalReached)

        return goalReached
