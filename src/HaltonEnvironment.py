#!/usr/bin/env python
# Take map and call the obstacle manager and pass it to the obstacle manager
# Create the successor function for different representations of the configuration space

import os
import math
import numpy
import GraphGenerator
import networkx as nx
from ObstacleManager import ObstacleManager


class HaltonEnvironment(object):

    def __init__(self, mapMsg, graphFile, source, target, car_width, car_length, neighbor_radius, collision_delta):

        # Setup member variables
        self.source = source
        self.target = target
        self.manager = ObstacleManager(mapMsg, car_width, car_length, collision_delta)
        # Generate the Graph on the fly if required
        self.radius = neighbor_radius
        if graphFile is None:
            n = 500
            bases = [2, 3]
            lower = [0, 0]
            upper = [64, 75]

            G = GraphGenerator.euclidean_halton_graph(n, self.radius, bases, lower, upper, source, target, mapMsg,
                                                      car_width, car_length, collision_delta)
            nx.write_graphml(G, "haltonGraph.graphml")
            self.graph = nx.read_graphml("haltonGraph.graphml")

        else:
            # Check if graph file exists
            if not os.path.isfile(graphFile):
                print
                "ERROR: map file not found!"
                quit()
            self.graph = nx.read_graphml(graphFile)

            if source is not None:
                GraphGenerator.insert_vertices(self.graph, [source], self.radius)

            if target is not None:
                GraphGenerator.insert_vertices(self.graph, [target], self.radius)

    def set_source_and_target(self, source, target):
        self.source = source
        self.target = target
        GraphGenerator.insert_vertices(self.graph, [source, target], self.radius)

    def get_config(self, vid):
        return [float(a) for a in self.graph.node[str(vid)]["state"].split()]

    def get_successors(self, vid):
        successors = [int(i) for i in self.graph.neighbors(str(vid))]
        return successors

    def get_state_validity(self, config2D):
        return self.manager.get_state_validity(config2D)

    def get_distance(self, vid1, vid2):
        G = self.graph
        return float(G[str(vid1)][str(vid2)]['length'])

    def get_heuristic(self, vid, tid):
        config1 = self.get_config(vid)
        config2 = self.get_config(tid)
        return numpy.linalg.norm(numpy.array(config1) - numpy.array(config2))
