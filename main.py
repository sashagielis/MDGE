from input_parser import read_ipe_instance
from visualizer import visualize


ipe_instances = {
    "simplified": [
        'simplified_instance_from_report',
    ],
    "MDGD": [
        'instance_from_report',
    ]
}


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
