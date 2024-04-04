from instance import SimplifiedInstance, GeneralInstance
from obstacle import PointObstacle
from point import Point
from visualizer import visualize

# IPE instances
ipe_instances = {
    "simplified": [
        #'simplified_instance_from_report',
        #'small test',
        'very large test',
        'very large test 2',
    ],
    "MDGD": [
        #'instance_from_report',
    ]
}


def main():
    for instance_name in ipe_instances["simplified"]:
        instance = SimplifiedInstance(instance_name, 'ipe')

        print(instance)
        visualize(instance, False, False)

        # Test visualization large vertices and thick edges
        for vertex in instance.graph.vertices:
            vertex.diameter = 20
        for edge in instance.graph.edges:
            edge.thickness = 20

        instance.solve('heuristic')

        print(instance)
        visualize(instance, True, False)

    for instance_name in ipe_instances["MDGD"]:
        instance = GeneralInstance(instance_name, 'ipe')

        print(instance)
        visualize(instance, False, False)

        # Test visualization large vertices and thick edges
        for vertex in instance.graph.vertices:
            vertex.diameter = 3
        for edge in instance.graph.edges:
            edge.thickness = 3

        instance.solve()

        print(instance)
        visualize(instance, True, False)


if __name__ == "__main__":
    main()
