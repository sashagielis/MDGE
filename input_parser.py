import os
import xml.etree.ElementTree as ET

from graph import *
from obstacle import *
from utils import transform_point


def read_ipe_instance(instance):
    vertices = []
    edges = []
    obstacles = []

    tree = ET.parse(f'instances/ipe/{instance}')
    root = tree.getroot()

    current_layer = None
    for obj in root.find('./page'):
        if obj.tag not in ['use', 'path']:
            continue

        if obj.get('layer') is not None:
            current_layer = obj.get('layer').casefold()

        if obj.get('matrix') is not None:
            transformation = [float(i) for i in obj.get('matrix').split()]
        else:
            transformation = [1, 0, 0, 1, 0, 0]

        match obj.tag:
            case 'use':
                pos = [float(i) for i in obj.get('pos').split()]
                point = Point(pos[0], pos[1])
                transform_point(point, transformation)

                color = obj.get('stroke')

                if current_layer == 'graph':
                    vertex = Vertex(point.x, point.y, color)
                    vertices.append(vertex)
                else:
                    obstacle = Obstacle([point], color)
                    obstacles.append(obstacle)

            case 'path':
                nodes_string = os.linesep.join([s for s in obj.text.splitlines() if s])
                path = []
                for node_string in nodes_string.split('\n'):
                    coordinates = [float(i) for i in node_string.split()[:-1]]
                    point = Point(coordinates[-2], coordinates[-1])
                    transform_point(point, transformation)
                    path.append(point)

                if current_layer == 'graph':
                    if obj.get('custom') is not None:
                        weight = obj.get('custom')
                    else:
                        weight = 1

                    edge = Edge(path, weight, obj.get('stroke'))
                    edges.append(edge)
                else:
                    if len(path) > 1:
                        path = path[:-1]

                    obstacle = Obstacle(path, obj.get('fill'), obj.get('stroke'))
                    obstacles.append(obstacle)

    graph = Graph(vertices, edges)

    return graph, obstacles
