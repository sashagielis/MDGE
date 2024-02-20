import os
import xml.etree.ElementTree as ET

from graph import *
from obstacle import *
from utils import transform_point
from visualizer import visualize


ipe_instances = {
    "simplified": [
        'simplified_instance_from_report',
    ],
    "MDGD": [
        'instance_from_report',
    ]
}


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
                x, y = transform_point(pos, transformation)

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
                    pos = coordinates[-2], coordinates[-1]
                    x, y = transform_point(pos, transformation)
                    path.append([x, y])

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


def main():
    for instance in ipe_instances["simplified"]:
        graph, obstacles = read_ipe_instance(f'{instance}.ipe')

        print("\n" + instance)
        print(graph)
        print("Obstacles:")
        for obstacle in obstacles:
            print(f"- {obstacle}")

        visualize(graph, obstacles, instance, False, True)

        # Test visualization large vertices and thick edges
        for vertex in graph.vertices:
            vertex.diameter = 3
        for edge in graph.edges:
            edge.thickness = 3
        visualize(graph, obstacles, instance, True, True)

    for instance in ipe_instances["MDGD"]:
        graph, obstacles = read_ipe_instance(f'{instance}.ipe')

        print("\n" + instance)
        print(graph)
        print("Obstacles:")
        for obstacle in obstacles:
            print(f"- {obstacle}")

        visualize(graph, obstacles, instance, False, False)

        # Test visualization large vertices and thick edges
        for vertex in graph.vertices:
            vertex.diameter = 3
        for edge in graph.edges:
            edge.thickness = 3
        visualize(graph, obstacles, instance, True, False)


if __name__ == "__main__":
    main()
