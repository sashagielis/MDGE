import os
import xml.etree.ElementTree as ET

from fractions import Fraction
from graph import *
from obstacle import *
from utils import transform_point


def read_ipe_instance(instance):
    """
    Reads an MDGD instance from an IPE file.

    :param instance: an IPE file consisting of a single page with the following layers:
    - 'graph': containing vertices (marks [M]) and edges (polylines [P])
    - 'obstacles': containing point obstacles (marks [M]) and/or polygonal obstacles (polylines [P])
    :returns: a Graph object and a list of Obstacle objects describing the MDGD instance
    """
    # Initialize Obstacle id counter
    Obstacle.id_iter = itertools.count()

    vertices = []
    edges = []
    obstacles = []

    tree = ET.parse(instance)
    root = tree.getroot()

    current_layer = None

    # Read each object as input
    for obj in root.find('./page'):
        if obj.tag not in ['use', 'path']:
            continue

        # Determine the current layer
        if obj.get('layer') is not None:
            current_layer = obj.get('layer').casefold()

        # Determine transformation applied by IPE
        if obj.get('matrix') is not None:
            transformation = [Fraction(i) for i in obj.get('matrix').split()]
        else:
            transformation = [1, 0, 0, 1, 0, 0]

        match obj.tag:
            case 'use':
                # The object is a mark
                pos = [float(i) for i in obj.get('pos').split()]
                point = Point(Fraction(pos[0]), Fraction(pos[1]))
                transform_point(point, transformation)

                if current_layer == 'graph':
                    # The mark is a vertex
                    vertex = Vertex(point.x, point.y, obj.get('stroke'))
                    vertices.append(vertex)
                else:
                    # The mark is a point obstacle
                    obstacle = PointObstacle(point, obj.get('stroke'))
                    obstacles.append(obstacle)

            case 'path':
                # The object is a path
                nodes_string = os.linesep.join([s for s in obj.text.splitlines() if s])
                path = []
                for node_string in nodes_string.split('\n'):
                    coordinates = [float(i) for i in node_string.split()[:-1]]
                    point = Point(Fraction(coordinates[-2]), Fraction(coordinates[-1]))
                    transform_point(point, transformation)
                    path.append(point)

                if current_layer == 'graph':
                    # The path is an edge
                    if obj.get('custom') is not None:
                        weight = float(obj.get('custom'))
                    else:
                        weight = 1

                    edge = Edge(path, weight, obj.get('stroke'))
                    edges.append(edge)
                else:
                    # The path is a polygonal obstacle
                    if len(path) == 1:
                        obstacle = PointObstacle(path[0], obj.get('fill'))
                    else:
                        path = path[:-1]
                        obstacle = PolygonalObstacle(path, obj.get('fill'), obj.get('stroke'))

                    obstacles.append(obstacle)

    # Replace first and last point of each edge by corresponding vertices
    for edge in edges:
        for vertex in vertices:
            if edge.path[0] == vertex:
                edge.path[0] = vertex

            if edge.path[-1] == vertex:
                edge.path[-1] = vertex

    graph = Graph(vertices, edges)

    return graph, obstacles
