from point import Point


class Vertex(Point):
    def __init__(self, x, y, color='black'):
        super().__init__(x, y)
        self.color = color
        self.diameter = 1


class Edge:
    def __init__(self, path, weight, color='black'):
        self.path = path
        self.weight = weight
        self.color = color
        self.thickness = 1

    def __str__(self):
        result = "["
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
