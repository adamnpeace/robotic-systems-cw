# -*- coding: utf-8 -*-

from math import *
from Queue import PriorityQueue
from cell_based_forward_search import CellBasedForwardSearch
from search_grid import SearchGrid
from planner_base import PlannerBase
from cell import *
from planned_path import PlannedPath
import rospy

class DijkstraPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid, useThoroughSearch=True):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.pq = PriorityQueue()
        # Gives us the option to continue even after reaching the goal,
        # which is how Dijkstra is implemented classically.
        self.useThoroughSearch = useThoroughSearch

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        if cell.coords == self.start.coords:
            cell.pathCost = 0
        self.pq.put((cell.pathCost, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.pq.empty()

    # Check the queue size
    def getQueueSize(self):
        return self.pq.qsize()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.pq.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        alt = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell)
        if alt < cell.pathCost:
            cell.parent = parentCell
            cell.pathCost = alt

    # Override `search()` method from `general_forward_search_algorithm`
    def search(self, startCoords, goalCoords):

        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()
        
        # Create or update the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (self.searchGrid is None):
            self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
        else:
            self.searchGrid.updateFromOccupancyGrid()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.label = CellLabel.GOAL

        # If the node is being shut down, bail out here.
        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0
        self.maxQueueSize = 0

        # Indicates if we reached the goal or not
        self.goalReached = False

        # Iterate until we have run out of live cells to try or we reached the goal.
        # This is the main computational loop and is the implementation of
        # LaValle's pseudocode
        while (self.isQueueEmpty() == False):

            self.maxQueueSize = max(self.maxQueueSize, self.getQueueSize())
            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging.
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                if not self.useThoroughSearch:
                    break
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    self.markCellAsVisitedAndRecordParent(nextCell, cell)
                    self.resolveDuplicate(nextCell, cell)
                    self.pushCellOntoQueue(nextCell)
                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                else:
                    self.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()
        
        print "numberOfCellsVisited = " + str(self.numberOfCellsVisited)

        print "maxQueueSize = " + str(self.maxQueueSize)
        
        if self.goalReached:
            print "Goal reached"
        else:
            print "Goal not reached"

        return self.goalReached
