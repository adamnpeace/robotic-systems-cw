import rospy
import math
from geometry_msgs.msg  import Pose2D
from nav_msgs.msg import Odometry

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self, detection_algorithm="fromRobot", heuristic="euclidean"):
        ExplorerNodeBase.__init__(self)

        self.blackList = []

        rospy.wait_for_message('/robot0/odom', Odometry)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Set the pose to an initial value to stop things crashing
        self.currentCoords = (0, 0)

        self.detection_algorithm = detection_algorithm
        self.heuristic = heuristic
    
    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        
        currentRWCoords= (position.x,position.y)
        self.currentCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(currentRWCoords)

    def updateFrontiers(self):
        pass

    def chooseNewDestinationFromOrigin(self):
        candidateGood = False
        destination = None
        smallestDist = float('inf')

        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    candidateGood = True
                    for k in range(0, len(self.blackList)):
                        if self.blackList[k] == candidate:
                            candidateGood = False
                            break
                    
                    if candidateGood is True:
                        if self.heuristic == "euclidean":
                            dist = math.sqrt(candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2)
                        elif self.heuristic == "manhattan":
                            dist = abs(candidate[0]) + abs(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())

                        if (dist < smallestDist):
                            destination = candidate
                            smallestDist = dist
        # If we got a good candidate, use it
        return candidateGood, destination

    def chooseNewDestinationFromRobot(self):
        candidateGood = False
        destination = None
        smallestDist = float('inf')

        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    candidateGood = True
                    for k in range(0, len(self.blackList)):
                        if self.blackList[k] == candidate:
                            candidateGood = False
                            break
                    
                    if candidateGood is True:
                        if self.heuristic == "euclidean":
                            dist = math.sqrt((candidate[0] - self.currentCoords[0])**2+(candidate[1] - self.currentCoords[0])**2)
                        elif self.heuristic == "manhattan":
                            dist = abs(candidate[0] - self.currentCoords[0]) + abs(candidate[1] - self.currentCoords[0])

                        if (dist < smallestDist):
                            destination = candidate
                            smallestDist = dist
        # If we got a good candidate, use it
        return candidateGood, destination

    def chooseNewDestination(self):
#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        if self.detection_algorithm == "fromRobot":
            return self.chooseNewDestinationFromRobot()
        elif self.detection_algorithm == "fromOrigin":
            return self.chooseNewDestinationFromOrigin()

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
