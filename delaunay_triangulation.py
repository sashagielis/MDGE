import numpy as np
from scipy.spatial import Delaunay

from utils import do_intersect


class DelaunayEdge:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.left_face = None
        self.right_face = None

    def __eq__(self, other):
        return (self.p1 == other.p1 and self.p2 == other.p2) or (self.p1 == other.p2 and self.p2 == other.p1)

    def __str__(self):
        return f"({self.p1}, {self.p2})"


class HalfLine:
    def __init__(self, p1, slope):
        self.p1 = p1
        self.slope = slope


class Face:
    def __init__(self):
        return


class Triangle(Face):
    def __init__(self):
        super().__init__()
        self.edges = []

    def get_edge(self, p1, p2):
        for edge in self.edges:
            if edge == DelaunayEdge(p1, p2):
                return edge

        return None

    def crossed_by(self, p, q):
        """
        Determines whether line segment pq crosses an edge of the triangle.

        :param p: a Point object
        :param q: a Point object
        :returns: the crossed Delaunay edge, or None if pq does not cross any
        """
        for edge in self.edges:
            if do_intersect(edge.origin, edge.target, p, q):
                return edge

        return None


class OuterFace(Face):
    def __init__(self):
        super().__init__()
        self.dt_edge = None
        self.half_lines = []


class DelaunayTriangulation:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance
        self.edges = []
        self.triangles = []
        self.outer_faces = []

        # Compute Delaunay triangulation on vertices and point obstacles
        vertex_points = [[float(vertex.x), float(vertex.y)] for vertex in instance.graph.vertices]
        obstacle_points = [[float(obstacle.x), float(obstacle.y)] for obstacle in instance.obstacles]
        delaunay_points = np.array(vertex_points + obstacle_points)
        dt = Delaunay(delaunay_points)

        n_v = len(vertex_points)
        n_triangles = len(dt.simplices)
        for i in range(n_triangles):
            triangle = Triangle()

            points = []
            for j in dt.simplices[i]:
                if j < n_v:
                    point = self.instance.graph.vertices[j]
                else:
                    point = self.instance.obstacles[j - n_v]

                points.append(point)

            for j in range(3):
                p1 = points[j]
                p2 = points[(j + 1) % 3]

                neighbor_index = dt.neighbors[i][(j + 2) % 3]
                if -1 < neighbor_index < i:
                    edge = self.triangles[neighbor_index].get_edge(p1, p2)
                    edge.right_face = triangle
                else:
                    edge = DelaunayEdge(p1, p2)
                    edge.left_face = triangle
                    self.edges.append(edge)

                    if neighbor_index == -1:
                        outer_face = OuterFace()
                        edge.right_face = outer_face
                        outer_face.dt_edge = edge
                        self.outer_faces.append(outer_face)

                triangle.edges.append(edge)

            self.triangles.append(triangle)

        # TODO: Add half lines to outer faces

    def __str__(self):
        for triangle in self.triangles:
            for edge in triangle.edges:
                print(edge, edge.left_face, edge.right_face)
