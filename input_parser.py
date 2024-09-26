import math
import os
import xml.etree.ElementTree as ET

from fractions import Fraction
from graph import *
from obstacle import *
from utils import transform_point

# Mapping from the colors of the MDGE IPE color stylesheet to their RGB values
mdge_ipe_color_stylesheet = {
    "MDGE lightblue": (141, 211, 199),
    "MDGE yellow": (255, 255, 179),
    "MDGE violet": (190, 186, 218),
    "MDGE red": (251, 128, 114),
    "MDGE blue": (128, 177, 211),
    "MDGE orange": (253, 180, 98),
    "MDGE green": (179, 222, 105),
    "MDGE pink": (252, 205, 229),
    "MDGE gray": (217, 217, 217),
    "MDGE purple": (188, 128, 189)
}


def parse_ipe_color(color):
    """
    Parses an IPE color.

    :param color: a color name or RGB values specified as "R G B" where each value is divided by 255
    :returns: the color name as string or a tuple with the RGB values of the color
    """
    # Check if the color is from the MDGE IPE color stylesheet
    if color in mdge_ipe_color_stylesheet:
        return mdge_ipe_color_stylesheet[color]

    color_list = color.split()

    # Check if the color is specified as its name or as RGB values
    if len(color_list) == 1:
        return color
    else:
        return tuple(int(float(c) * 255) for c in color_list)


def read_ipe_instance(instance):
    """
    Reads an MDGE instance from an IPE file.

    :param instance: an IPE file consisting of a single page with the following layers:
    - 'graph': containing vertices (marks [M]) and edges (polylines [P])
    - 'obstacles': containing point obstacles (marks [M] or circles [O]) and/or polygonal obstacles (polylines [P])
    :returns: a Graph object, a list of Obstacle objects and the x- and y-ranges of the instance
    """
    # Initialize Vertex and Obstacle id counters
    Vertex.id_iter = itertools.count()
    Obstacle.id_iter = itertools.count()

    vertices = []
    edges = []
    obstacles = []

    # Initialize instance dimensions
    x_range = [math.inf, -math.inf]
    y_range = [math.inf, -math.inf]

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

        # Parse the stroke color
        stroke_color = parse_ipe_color(obj.get('stroke'))

        match obj.tag:
            case 'use':
                # The object is a mark
                pos = [float(i) for i in obj.get('pos').split()]
                point = Point(Fraction(pos[0]), Fraction(pos[1]))
                transform_point(point, transformation)

                # Update instance dimensions
                x_range = [min(x_range[0], point.x), max(x_range[1], point.x)]
                y_range = [min(y_range[0], point.y), max(y_range[1], point.y)]

                if current_layer == 'graph':
                    # The mark is a vertex
                    vertex = Vertex(point.x, point.y, stroke_color)
                    vertices.append(vertex)
                else:
                    # The mark is a point obstacle
                    obstacle = PointObstacle(point, stroke_color)
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

                    # Update instance dimensions
                    x_range = [min(x_range[0], point.x), max(x_range[1], point.x)]
                    y_range = [min(y_range[0], point.y), max(y_range[1], point.y)]

                if current_layer == 'graph':
                    # The path is an edge
                    if obj.get('custom') is not None:
                        weight = float(obj.get('custom'))
                    else:
                        weight = 1

                    edge = Edge(path, weight, stroke_color)
                    edges.append(edge)
                else:
                    # Parse the fill color
                    fill_color = parse_ipe_color(obj.get('fill'))

                    # The path is an obstacle
                    if len(path) == 1:
                        obstacle = PointObstacle(path[0], fill_color)
                    else:
                        path = path[:-1]
                        obstacle = PolygonalObstacle(path, fill_color, stroke_color)

                    obstacles.append(obstacle)

    # Replace first and last point of each edge by corresponding vertices
    for edge in edges:
        for vertex in vertices:
            if edge.path[0] == vertex:
                edge.v1 = vertex
                edge.path[0] = vertex
                vertex.radius = edge.thickness / 2

            if edge.path[-1] == vertex:
                edge.v2 = vertex
                edge.path[-1] = vertex
                vertex.radius = edge.thickness / 2

        if edge.v1 is None or edge.v2 is None or edge.v1 == edge.v2:
            edges.remove(edge)

    graph = Graph(vertices, edges)

    return graph, obstacles, x_range, y_range
