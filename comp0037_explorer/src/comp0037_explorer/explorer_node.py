import rospy
import math
from frontier import Frontier

from explorer_node_base import ExplorerNodeBase

class Cell(object):

    def __init__(self, coords):

        # Set coordinates
        self.coords = coords


# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self, heuristic="width"):
        ExplorerNodeBase.__init__(self)

        self.blackList = []

        self.heuristic = heuristic
    
        
    def getNextCells(self, cell):

        # self stores the set of valid actions / cells
        cells = list();

        # Go through all the neighbours and add the cells if they
        # don't fall outside the grid and they aren't the cell we
        # started with. The order has been manually written down to
        # create a spiral.
        self.pushBackCandidateCellIfValid(cell, cells, 0, -1)
        self.pushBackCandidateCellIfValid(cell, cells, 1, -1)
        self.pushBackCandidateCellIfValid(cell, cells, 1, 0)
        self.pushBackCandidateCellIfValid(cell, cells, 1, 1)
        self.pushBackCandidateCellIfValid(cell, cells, 0, 1)
        self.pushBackCandidateCellIfValid(cell, cells, -1, 1)
        self.pushBackCandidateCellIfValid(cell, cells, -1, 0)
        self.pushBackCandidateCellIfValid(cell, cells, -1, -1)

        return cells

    # This helper method checks if the robot, at cell.coords, can move
    # to cell.coords+(offsetX, offsetY). Reasons why it can't do this
    # include falling off the edge of the map or running into an
    # obstacle.
    def pushBackCandidateCellIfValid(self, cell, cells, offsetX, offsetY):
        newX = cell.coords[0] + offsetX
        newY = cell.coords[1] + offsetY
        extent = self.occupancyGrid.getExtentInCells()
        if ((newX >= 0) & (newX < extent[0]) \
            & (newY >= 0) & (newY < extent[1])):
            if self.occupancyGrid.getCell(newX, newY) != 0:
                return False
            newCoords = (newX, newY)
            newCell = Cell(newCoords)
            if not self.occupancyGrid.getCell(newX, newY) == 0.5:
                cells.append(newCell)
                
    def isIn(self, cell, visited):
        for v in visited:
            if cell.coords[0] == v.coords[0] and cell.coords[1] == v.coords[1]:
                return True
        return False

    def isInFrontiers(self, cell):
        for frontier in self.frontiers:
            if self.isIn(cell, frontier.nodes):
                return True
        return False

    def pushBackCandidateCellIfFrontier(self, cell, cells, offsetX, offsetY):
        newX = cell.coords[0] + offsetX
        newY = cell.coords[1] + offsetY
        extent = self.occupancyGrid.getExtentInCells()
        if ((newX >= 0) & (newX < extent[0]) \
            & (newY >= 0) & (newY < extent[1])):
            if self.occupancyGrid.getCell(newX, newY) != 0:
                return False
            newCoords = (newX, newY)
            newCell = Cell(newCoords)
            if self.isFrontierCell(newX, newY) and not self.isInFrontiers(newCell):
                cells.append(newCell)

    def getNextFrontierCells(self, cell):
        # self stores the set of valid actions / cells
        cells = list();

        # Go through all the neighbours and add the cells if they
        # don't fall outside the grid and they aren't the cell we
        # started with. The order has been manually written down to
        # create a spiral.
        self.pushBackCandidateCellIfFrontier(cell, cells, 0, -1)
        self.pushBackCandidateCellIfFrontier(cell, cells, 1, -1)
        self.pushBackCandidateCellIfFrontier(cell, cells, 1, 0)
        self.pushBackCandidateCellIfFrontier(cell, cells, 1, 1)
        self.pushBackCandidateCellIfFrontier(cell, cells, 0, 1)
        self.pushBackCandidateCellIfFrontier(cell, cells, -1, 1)
        self.pushBackCandidateCellIfFrontier(cell, cells, -1, 0)
        self.pushBackCandidateCellIfFrontier(cell, cells, -1, -1)

        return cells

    def frontierDFS(self, startCell):
        newFrontier = Frontier()
        lq = list()
        visited = list()
        visited.append(startCell)
        lq.append(startCell)

        while len(lq) != 0:
            cell = lq[0]
            lq = lq[1:]

            newFrontier.nodes.append(cell)

            newCells = self.getNextFrontierCells(cell)
            for newCell in newCells:
                if not self.isIn(newCell, newFrontier.nodes) and not self.isIn(newCell, lq) and not self.isInFrontiers(newCell):
                    visited.append(newCell)
                    lq.append(newCell)
        self.frontiers.append(newFrontier)

    def updateFrontiers(self):
        self.frontiers = []
        lq = list()
        visited = list()
        start = Cell((10, int(0.5*self.occupancyGrid.getHeightInCells())))
        visited.append(start)
        lq.append(start)

        while len(lq) != 0:
            cell = lq[0]
            lq = lq[1:]

            self.frontierDFS(cell)

            newCells = self.getNextCells(cell)
            for newCell in newCells:
                if not self.isIn(newCell, visited) and not self.isIn(newCell, lq) and not self.isInFrontiers(newCell):
                    visited.append(newCell)
                    lq.append(newCell)
        return self.frontiers != []

    def chooseNewDestination(self):
        self.updateFrontiers()
        candidateGood = bool(self.frontiers)
        destination = None
        largestWidth = 0
        smallestDist = float('inf')
        for frontier in self.frontiers:
            candidate = (frontier.center().coords[0], frontier.center().coords[1])
            if candidate in self.blackList:
                continue
            if self.heuristic == "width" and frontier.width() > largestWidth:
                destination = candidate
                largestWidth = frontier.width()
            elif self.heuristic == "euclidean":
                dist = math.sqrt(candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2)
                if (dist < smallestDist):
                    destination = candidate
                    smallestDist = dist
        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
