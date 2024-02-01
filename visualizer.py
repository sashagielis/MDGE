import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle


def visualize_input(graph, obstacles, instance):
    fig, ax = plt.subplots()

    for edge in graph.edges:
        path = edge.path
        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], linewidth=1, color=edge.color, zorder=1)

    for vertex in graph.vertices:
        v = Circle((vertex.x, vertex.y), 1, color=vertex.color, zorder=2)
        ax.add_patch(v)

    for obstacle in obstacles:
        path = obstacle.path
        if len(path) == 1:
            o = Circle(path[0], 1, color=obstacle.fill_color, zorder=3)
        else:
            o = Polygon(path, edgecolor=obstacle.stroke_color, facecolor=obstacle.fill_color, zorder=3)
        ax.add_patch(o)

    plt.axis('scaled')
    plt.axis('off')
    plt.savefig(f'plots/input/{instance}.png', bbox_inches='tight')
