#!/usr/bin/env python

# Import standard python libraries
import rospy
from nav_msgs.srv import GetMap
import networkx as nx
import math
import numpy
from scipy import spatial
from ObstacleManager import ObstacleManager
import os


# Halton Sequence Generator
def halton_sequence_value(index, base):
    result = 0
    f = 1

    while index > 0:
        f = f * 1.0 / base
        result = result + f * (index % base)
        index = index / base

    return result


# Wrap the values around 0 and 1
def wrap_around(coordinate):
    for i in range(numpy.size(coordinate)):
        if coordinate[i] > 1.0:
            coordinate[i] = coordinate[i] - 1.0
        if coordinate[i] < 0:
            coordinate[i] = 1.0 + coordinate[i]

    return coordinate


# Halton Graph Generator
def euclidean_halton_graph(n, radius, bases, lower, upper, source, target, mapFile, car_width, car_length,
                           collision_delta):
    manager = ObstacleManager(mapFile, car_width, car_length, collision_delta)

    G = nx.DiGraph()
    upper = numpy.array(upper)
    lower = numpy.array(lower)
    scale = upper - lower
    offset = lower

    position = []

    numVertices = 0
    haltonIndex = 1

    if source is not None:
        position.append(source)
        num_vertices += 1
    if target is not None:
        position.append(target)
        num_vertices += 1

    print '[GraphGenerator] Populating node...'
    while numVertices < n:
        p = wrap_around(numpy.array([halton_sequence_value(haltonIndex, base) for base in bases]))
        p = p * scale + offset

        if manager.get_state_validity(p):
            position.append(p)
            numVertices += 1

        haltonIndex += 1

    state = [" ".join(str(x) for x in p) for p in position]

    for i in range(n):
        node_id = i
        G.add_node(str(node_id), state=state[i])

    print '[Graph Generator] Generating KD Tree...'
    position = numpy.array(position)
    tree = spatial.KDTree(position)

    print '[GraphGenerator] Populating edges...'
    for i in xrange(n):
        distances, indices = tree.query(position[numpy.newaxis, i], k=100, distance_upper_bound=radius)
        distances = distances.squeeze()
        indices = indices.squeeze()

        edgeCount = 0
        for j in xrange(indices.shape[0]):
            if indices[j] >= len(position):
                break
            if distances[j] > numpy.finfo(float).eps:
                edgeLength = numpy.linalg.norm(position[i] - position[indices[j]])
                G.add_edge(str(i), str(indices[j]), length=str(edgeLength))

                edgeCount += 1
        print '[GraphGenerator] %d of %d nodes complete, edges: %d' % (i, n, edgeCount)

    print '[GraphGenerator] Graph generation complete'
    return G


def insert_vertices(G, configs, radius):
    numVertices = G.number_of_nodes()
    for config in configs:
        state = " ".join(str(x) for x in config)
        G.add_node(str(numVertices), state=state)
        for i in range(numVertices):
            position = numpy.array([float(a) for a in G.node[str(i)]["state"].split()])
            edgeLength = numpy.linalg.norm(numpy.array(config) - position)
            if edgeLength < radius:
                G.add_edge(str(numVertices), str(i), length=str(edgeLength))
                G.add_edge(str(i), str(numVertices), length=str(edgeLength))

        numVertices += 1

    # nx.write_graphml(G, "currentHalton.graphml")


def generate_graph_file(map_msg, halton_points, disc_radius, car_width, car_length, collision_delta):
    file_dir = os.path.expanduser('~/.ros/halton_graph_files')
    file_name = (str(int(numpy.array(map_msg.data).sum())) + "_" +
                 str(int(halton_points)) + "_" +
                 str(int(disc_radius)) + "_" +
                 str(int(1000 * map_msg.info.resolution + 0.5)) + "_" +
                 str(int(map_msg.info.width)) + "_" +
                 str(int(map_msg.info.height)) + "_" +
                 str(int(1000 * map_msg.info.origin.position.x + 0.5)) + "_" +
                 str(int(1000 * map_msg.info.origin.position.y + 0.5)) + "_" +
                 str(int(1000 * map_msg.info.origin.position.z + 0.5)) + "_" +
                 str(int(1000 * map_msg.info.origin.orientation.x + 0.5)) + "_" +
                 str(int(1000 * map_msg.info.origin.orientation.y + 0.5)) + "_" +
                 str(int(1000 * map_msg.info.origin.orientation.z + 0.5)) + "_" +
                 str(int(1000 * map_msg.info.origin.orientation.w + 0.5)) + '.graphml')
    file_name = file_dir + '/' + file_name

    if not os.path.isdir(file_dir):
        os.makedirs(file_dir)

    if not os.path.exists(file_name):
        bases = [2, 3]
        lower = numpy.array([map_msg.info.origin.position.x, map_msg.info.origin.position.y])
        upper = numpy.array([map_msg.info.origin.position.x + map_msg.info.resolution * map_msg.info.width,
                             map_msg.info.origin.position.y + map_msg.info.resolution * map_msg.info.height])

        numpy.random.seed(0)
        offset = numpy.random.random_sample(len(bases), )
        G = euclidean_halton_graph(halton_points, disc_radius, bases, lower, upper, None, None, map_msg, car_width,
                                   car_length, collision_delta)
        nx.write_graphml(G, file_name)

    return file_name


# Main Function
if __name__ == "__main__":
    rospy.init_node("generate_graph")
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)

    graph_file = rospy.get_param("~graph_file", None)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
    map_info = map_msg.info

    spaceDimension = 2

    if spaceDimension == 2:
        bases = [2, 3]

    lower = numpy.array([map_info.origin.position.x, map_info.origin.position.y])
    upper = numpy.array([map_info.origin.position.x + map_info.resolution * map_info.width,
                         map_info.origin.position.y + map_info.resolution * map_info.height])

    # Settings
    halton_points = 500
    disc_radius = 100  # 2*halton_points**(-0.5)
    print(disc_radius)

    for i in range(1):
        print i
        numpy.random.seed(0)
        offset = numpy.random.random_sample(spaceDimension, )
        riskmapFile = 'haltonGraph.graphml'

        # Generate the graph
        print 'Generating the graph'
        G = euclidean_halton_graph(halton_points, disc_radius, bases, lower, upper, None, None, map_msg)
        nx.write_graphml(G, riskmapFile)
