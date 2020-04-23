# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def findFirstObstacleCellOnPath(self):
        # Returns index of obstacle in current path
        for i, cell in enumerate(self.currentPlannedPath.waypoints):
            if self.occupancyGrid.getCell(cell.coords[0], cell.coords[1]) == 1:
                return i
        else:
            return -1

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()
        if self.findFirstObstacleCellOnPath() > -1:
            self.controller.stopDrivingToCurrentGoal()

    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):
        return Aisle.C

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.C

    def isWaypointObstacle(self, waypoint):
        return self.occupancyGrid.getCell(w.coords[0], w.coords[1]) == 1

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):

        obstacleIndex = self.findFirstObstacleCellOnPath()
        if obstacleIndex == -1:
            return True

        for currentCellindex, cell in enumerate(self.currentPlannedPath.waypoints):
            if cell.coords == (startCellCoords, goalCellCoords):
                break

        
        waitCost = 2 + (obstacleIndex - currentCellindex)

        obstacleCell = self.currentPlannedPath.waypoints[obstacleIndex]
        obstacle = obstacleCell.coords[0], obstacleCell.coords[1]

        newAisle = self.chooseAisle(*obstacle)
        reroutePath = self.planPathToGoalViaAisle(*obstacle, aisle=newAisle)
        rerouteCost = len(reroutePath.waypoints)

        if waitCost < rerouteCost:
            return True
        else:
            return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):
        while True:
            for w in self.currentPlannedPath.waypoints:
                if self.isWaypointObstacle(w):
                    break
            else:
                return
    
    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle

        

        # Construct a path which will drive the robot
        # from the start to the goal via the aisle.
        aisleCoords = [
            (133/5, 75/5),
            (214/5, 75/5),
            (294/5, 75/5),
            (372/5, 75/5),
            (449/5, 75/5)
        ]

        aisleCellCoords = aisleCoords[aisle]

        pathToAisleFound = self.planner.search(startCellCoords, aisleCellCoords)
        
        # Extract the path to aisle center
        currentPlannedPath = self.planner.extractPathToGoal()

        pathToGoalFound = self.planner.search(aisleCellCoords, goalCellCoords)

        # If we can't reach the goal, give up and return
        if not pathToGoalFound or not pathToAisleFound:
            rospy.logwarn("Could not find a path to the goal at (%d, %d) through (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1], aisleCellCoords[0], aisleCellCoords[1])
            return None

        aisleToGoalPath = self.planner.extractPathToGoal()

        [currentPlannedPath.waypoints.append(b) for b in aisleToGoalPath.waypoints]
        currentPlannedPath.travelCost += aisleToGoalPath.travelCost
        currentPlannedPath.numberOfWaypoints += aisleToGoalPath.numberOfWaypoints

        print 'final travel cost: ' + str(currentPlannedPath.travelCost)
        print 'final waypoints num: ' + str(currentPlannedPath.numberOfWaypoints)

        self.planner.displayFullPath(currentPlannedPath, 'yellow')

        return currentPlannedPath

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        # Or change the next line to select and isle
        aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)
        # aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)

            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
