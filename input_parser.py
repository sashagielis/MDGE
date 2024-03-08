import os
import xml.etree.ElementTree as ET

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
    vertices = []
    edges = []
    obstacles = []

    tree = ET.parse(f'instances/ipe/{instance}')
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
            transformation = [float(i) for i in obj.get('matrix').split()]
        else:
            transformation = [1, 0, 0, 1, 0, 0]

        match obj.tag:
            case 'use':
                # The object is a mark
                pos = [float(i) for i in obj.get('pos').split()]
                point = Point(pos[0], pos[1])
                transform_point(point, transformation)

                color = obj.get('stroke')

                if current_layer == 'graph':
                    # The mark is a vertex
                    vertex = Vertex(point.x, point.y, color)
                    vertices.append(vertex)
                else:
                    # The mark is a point obstacle
                    obstacle = Obstacle([point], color)
                    obstacles.append(obstacle)

            case 'path':
                # The object is a path
                nodes_string = os.linesep.join([s for s in obj.text.splitlines() if s])
                path = []
                for node_string in nodes_string.split('\n'):
                    coordinates = [float(i) for i in node_string.split()[:-1]]
                    point = Point(coordinates[-2], coordinates[-1])
                    transform_point(point, transformation)
                    path.append(point)

                if current_layer == 'graph':
                    # The path is an edge
                    if obj.get('custom') is not None:
                        weight = obj.get('custom')
                    else:
                        weight = 1

                    edge = Edge(path, weight, obj.get('stroke'))
                    edges.append(edge)
                else:
                    # The path is a polygonal obstacle
                    if len(path) > 1:
                        path = path[:-1]

                    obstacle = Obstacle(path, obj.get('fill'), obj.get('stroke'))
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
