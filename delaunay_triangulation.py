import numpy as np
from scipy.spatial import Delaunay

from point import Point
from utils import check_segment_segment_intersection, orientation


class DelaunayVertex(Point):
    """
    A vertex of the Delaunay triangulation.
    """
    def __init__(self, point):
        """
        :param point: a Point object
        """
        super().__init__(point.x, point.y)

        self.outgoing_edges = []  # Set of half-edges leaving the vertex


class HalfEdge:
    """
    A half-edge connecting two Delaunay points, contained within the triangle on its left-hand side.
    """
    def __init__(self, origin, target):
        """
        :param origin: a Point object, specifying the origin of the half-edge
        :param target: a Point object, specifying the target of the half-edge
        """
        self.origin = origin
        self.target = target
        self.twin = None  # The reverse half-edge directed from target to origin
        self.next = None  # The next half-edge along the same triangle
        self.prev = None  # The previous half-edge along the same triangle
        self.triangle = None  # The triangle containing the half-edge

    def orientation(self, p):
        """
        Determines the orientation of the point with respect to the half-edge.

        :returns: 0 : collinear, 1 : clockwise, 2 : counterclockwise
        """
        return orientation(self.origin, self.target, p)

    def intersects(self, p, q):
        """
        Determines whether the half-edge intersects line segment pq.
        """
        if check_segment_segment_intersection(self.origin, self.target, p, q):
            return True
        else:
            return False

    def __str__(self):
        return f"{self.origin} -> {self.target}"


class Triangle:
    """
    A triangle of the Delaunay triangulation.
    """
    def __init__(self):
        self.half_edges = []  # Set of half-edges forming the triangle

    def get_edge(self, origin, target):
        """
        Returns the edge in the triangle with given origin and target, or None if it does not exist.
        """
        for he in self.half_edges:
            if he.origin == origin and he.target == target:
                return he

        return None

    def exited_by(self, p, q):
        """
        Determines whether an edge exits the triangle via its link pq.
        Checks if the directed line segment pq crosses one of the triangles' half-edges towards the outside.

        :param p: a Point object
        :param q: a Point object
        :returns: the crossed half-edge (either if crossed in a node), or None if pq does not exit the triangle
        """
        for he in self.half_edges:
            if (he.intersects(p, q)
                    and (he.orientation(p) == 2 != he.orientation(q)
                         or (he.orientation(p) == 0 and he.orientation(q) == 1))):
                return he

        return None

    def __str__(self):
        he = self.half_edges[0]

        return f"{he.origin} -> {he.target} -> {he.next.target}"


class DelaunayTriangulation:
    """
    A Delaunay triangulation on a set of points.
    """
    def __init__(self, points):
        """
        :param points: a list of Point objects
        """
        self.vertices = []
        self.point_to_dt_vertex = {}
        dt_input_points = []

        # Construct Delaunay vertices
        for point in points:
            dt_vertex = DelaunayVertex(point)
            self.vertices.append(dt_vertex)
            self.point_to_dt_vertex[id(point)] = dt_vertex

            dt_input_points.append([float(point.x), float(point.y)])

        self.triangles = []

        # Compute the Delaunay triangulation
        dt = Delaunay(np.array(dt_input_points))

        # Iterate over all triangles
        for i in range(len(dt.simplices)):
            triangle = Triangle()

            # Consider each edge of the triangle
            for j in range(3):
                p1 = self.vertices[dt.simplices[i][j]]
                p2 = self.vertices[dt.simplices[i][(j + 1) % 3]]

                # Construct half-edge
                he = HalfEdge(p1, p2)
                he.triangle = triangle
                p1.outgoing_edges.append(he)

                # Find other triangle adjacent to the edge
                neighbor_index = dt.neighbors[i][(j + 2) % 3]

                # If neighboring triangle was already created, find twin edge
                if -1 < neighbor_index < i:
                    twin = self.triangles[neighbor_index].get_edge(p2, p1)

                    # Set twin
                    he.twin = twin
                    twin.twin = he

                triangle.half_edges.append(he)

            # Set next and prev
            for j in range(3):
                triangle.half_edges[j].next = triangle.half_edges[(j + 1) % 3]
                triangle.half_edges[j].prev = triangle.half_edges[(j - 1) % 3]

            self.triangles.append(triangle)

    def get_delaunay_vertex_from_point(self, point):
        """
        Maps a point to its corresponding Delaunay vertex.

        :param point: a Point object
        :returns: the DelaunayVertex object corresponding to the given point
        """
        if id(point) in self.point_to_dt_vertex:
            return self.point_to_dt_vertex[id(point)]
        else:
            raise Exception(f"Given point {point} is not a Delaunay vertex")

    def __str__(self):
        result = "Triangles:\n"
        for triangle in self.triangles:
            result += f"- {triangle}\n"

        result = result[:-1]

        return result
