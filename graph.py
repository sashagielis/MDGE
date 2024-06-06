import itertools

from point import Point


class Vertex(Point):
    id_iter = itertools.count()

    def __init__(self, x, y, color='black'):
        super().__init__(x, y)
        self.id = next(self.id_iter)
        self.color = color
        self.radius = 0.5

        self.elbow_bundles = []  # List of elbow bundles associated with the vertex, i.e., bending around it


class Edge:
    def __init__(self, path, weight, color='black'):
        self.v1 = path[0]
        self.v2 = path[-1]
        self.path = path
        self.weight = weight
        self.color = color
        self.thickness = 1

        self.crossing_sequence = None  # The reduced crossing sequence of the edge through the Delaunay triangulation
        self.elbow_bundle_v1 = None  # The terminal elbow bundle for v1

    def __str__(self):
        result = f"({self.color}) ["
        for point in self.path:
            result += str(point) + ", "
        result = result[:-2] + "]"

        return result + f", weight = {self.weight}"


class Graph:
    def __init__(self, vertices, edges):
        self.vertices = vertices
        self.edges = edges

    def __str__(self):
        result = "Vertices: "
        for vertex in self.vertices:
            result += str(vertex) + ", "
        result = result[:-2]

        result += "\nEdges:\n"
        for edge in self.edges:
            result += "- " + str(edge) + "\n"
        result = result[:-1]

        return result
