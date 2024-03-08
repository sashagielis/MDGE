from instance import SimplifiedInstance, GeneralInstance
from visualizer import visualize

# IPE instances
ipe_instances = {
    "simplified": [
        'simplified_instance_from_report',
        'test',
    ],
    "MDGD": [
        'instance_from_report',
    ]
}


def main():
    for instance_name in ipe_instances["simplified"]:
        instance = SimplifiedInstance(instance_name, 'ipe')

        print(instance)
        visualize(instance, False, False)

        # Test visualization large vertices and thick edges
        for vertex in instance.graph.vertices:
            vertex.diameter = 15
        for edge in instance.graph.edges:
            edge.thickness = 15

        instance.solve()

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

        visualize(instance, True, False)


if __name__ == "__main__":
    main()
