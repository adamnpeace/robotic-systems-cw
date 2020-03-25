# -*- coding: utf-8 -*-

# This class stores information about each frontier

class Frontier(object):

    def __init__(self):

        self.nodes = []
        
        self.visited = 0

    def width(self):
        return len(self.nodes)

    def center(self):
        return self.nodes[len(self.nodes)/2]