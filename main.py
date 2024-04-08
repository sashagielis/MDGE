from instance import SimplifiedInstance, GeneralInstance
from visualizer import visualize

# IPE instances
ipe_instances = {
    "simplified": [
        # 'simplified_instance_from_report',
        # 'One line small',
        # 'One line large',
        # 'Two lines',
        # 'Curve edge case',
        'Two lines edge case',
    ],
    "MDGD": [
        # 'instance_from_report',
    ]
}


def main():
    for instance_name in ipe_instances["simplified"]:
        instance = SimplifiedInstance(instance_name, 'ipe')

        print(instance)
        visualize(instance, False, False)

        # Test visualization large vertices and thick edges
        # for vertex in instance.graph.vertices:
        #     vertex.diameter = 100
        for edge in instance.graph.edges:
            edge.thickness = edge.weight

        instance.solve(displacement_method='iterative')

        print(instance)
        visualize(instance, True, False)


if __name__ == "__main__":
    main()
