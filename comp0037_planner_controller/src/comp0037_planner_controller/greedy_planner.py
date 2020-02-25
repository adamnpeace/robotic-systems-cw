# -*- coding: utf-8 -*-

from math import *
from Queue import PriorityQueue 
from cell_based_forward_search import CellBasedForwardSearch

class GreedyPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.pq = PriorityQueue()

    def getEuclideanToGoal(self, cell):
        dX = cell.coords[0] - self.goal.coords[0]
        dY = cell.coords[1] - self.goal.coords[1]

        L = sqrt(dX * dX + dY * dY)

        return L

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.pq.put((self.getEuclideanToGoal(cell), cell))

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
        # Nothing to do in self case
        pass
