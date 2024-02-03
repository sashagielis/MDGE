class Vertex:
    def __init__(self, x, y, color='black'):
        self.x = x
        self.y = y
        self.color = color
        self.diameter = 1

    def __str__(self):
        return f"({self.x}, {self.y})"


class Edge:
    def __init__(self, path, weight, color='black'):
        self.path = path
        self.weight = weight
        self.color = color
        self.thickness = 1

    def __str__(self):
        result = "["
        for node in self.path:
            result += f"({node[0]}, {node[1]}), "
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
