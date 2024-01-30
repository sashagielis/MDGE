import os
import xml.etree.ElementTree as ET

from graph import *
from obstacle import *


def read_ipe_file(filename):
    vertices = []
    edges = []
    obstacles = []

    tree = ET.parse(f'instances/{filename}')
    root = tree.getroot()

    current_layer = None
    for obj in root.find('./page'):
        if obj.tag not in ['use', 'path']:
            continue

        if obj.get('layer') is not None:
            current_layer = obj.get('layer').casefold()

        if obj.get('matrix') is not None:
            transform_matrix = [float(i) for i in obj.get('matrix').split()]
        else:
            transform_matrix = [1, 0, 0, 1, 0, 0]

        match obj.tag:
            case 'use':
                pos = [float(i) for i in obj.get('pos').split()]
                x, y = transform(pos, transform_matrix)

                color = obj.get('stroke')

                if current_layer == 'graph':
                    vertex = Vertex(x, y, color)
                    vertices.append(vertex)
                else:
                    obstacle = Obstacle([[x, y]], color)
                    obstacles.append(obstacle)

            case 'path':
                nodes_string = os.linesep.join([s for s in obj.text.splitlines() if s])
                path = []
                for node_string in nodes_string.split('\n'):
                    coordinates = [float(i) for i in node_string.split()[:-1]]
                    #print(coordinates)
                    pos = coordinates[-2], coordinates[-1]
                    x, y = transform(pos, transform_matrix)
                    path.append([x, y])

                if current_layer == 'graph':
                    if obj.get('custom') is not None:
                        weight = obj.get('custom')
                    else:
                        weight = 1

                    edge = Edge(path, weight, obj.get('stroke'))
                    edges.append(edge)
                else:
                    obstacle = Obstacle(path, obj.get('fill'))
                    obstacles.append(obstacle)

    graph = Graph(vertices, edges)

    return graph, obstacles


def transform(p, t):
    """
    Transforms point p using scaling matrix and translation vector given by t.

    :param p: [x, y]
    :param t: [m11, m12, m21, m22, t1, t2]
    :return: [[m11, m12], [m21, m22]] * [x, y] + [t1, t2]
    """
    return [t[0] * p[0] + t[1] * p[1] + t[4], t[2] * p[0] + t[3] * p[1] + t[5]]


def main():
    graph, obstacles = read_ipe_file('instance_from_report.ipe')
    print(graph)

    print("\nObstacles:")
    for obstacle in obstacles:
        print(f"- {obstacle}")


if __name__ == "__main__":
    main()
