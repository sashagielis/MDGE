import numpy as np
from scipy.spatial import Delaunay

from graph import Vertex
from obstacle import PointObstacle
from point import Point
from utils import check_segment_half_line_intersection, check_segment_segment_intersection, vector_bisector, orientation


class DelaunayVertex(Point):
    """
    A vertex of the Delaunay triangulation representing a Vertex or PointObstacle.
    """
    def __init__(self, point):
        """
        :param point: a Vertex or PointObstacle object
        """
        super().__init__(point.x, point.y)

        self.point = point
        self.outgoing_edges = []  # Set of Delaunay edges leaving the vertex


class HalfEdge:
    """
    A half-edge connecting two DelaunayVertex objects, contained within the face on its left-hand side.
    """
    def __init__(self, origin, target):
        """
        :param origin: a Point object, specifying the origin of the half-edge
        :param target: a Point object, specifying the target of the half-edge
        """
        self.origin = origin
        self.target = target
        self.twin = None  # The reverse half-edge directed from target to origin
        self.next = None  # The next Delaunay edge along the same face
        self.prev = None  # The previous Delaunay edge along the same face
        self.face = None  # The face containing the half-edge

    def orientation(self, p):
        """
        Determines the orientation of the point with respect to the half-edge.

        :returns: 0 : collinear, 1 : clockwise, 2 : counterclockwise
        """
        return orientation(self.origin, self.target, p)

    def bisector(self, other):
        """
        Determines the bisector between two half-edges.
        """
        vec1 = self.target - self.origin
        vec2 = other.target - other.origin

        return vector_bisector(vec1, vec2)

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


class HalfLine:
    """
    A half-line originated in a DelaunayVertex object, contained within the outer face on its left-hand side.
    """
    def __init__(self, origin, other_point, rev=False):
        """
        :param origin: a Point object, specifying the origin of the half-line
        :param other_point: a Point object, specifying some point on the half-line other than the origin
        :param rev: whether the half-line is reversed, i.e., is directed towards its origin
        """
        self.origin = origin
        self.other_point = other_point
        self.reversed = rev
        self.twin = None  # The reverse half-line
        self.next = None  # The next Delaunay edge along the same face, may be None
        self.prev = None  # The previous Delaunay edge along the same face, may be None
        self.face = None  # The outer face containing the half-line

    def orientation(self, p):
        """
        Determines the orientation of the point with respect to the half-line.

        :returns: 0 : collinear, 1 : clockwise, 2 : counterclockwise
        """
        if self.reversed:
            return orientation(self.other_point, self.origin, p)
        else:
            return orientation(self.origin, self.other_point, p)

    def intersects(self, p, q):
        """
        Determines whether the half-line intersects line segment pq.
        """
        if check_segment_half_line_intersection(self.origin, self.other_point, p, q):
            return True
        else:
            return False

    def __str__(self):
        if self.reversed:
            return f"inf -> {self.origin}"
        else:
            return f"{self.origin} -> inf"


class Face:
    """
    A face of the subdivision.
    """
    def __init__(self):
        self.edges = []  # Set of Delaunay edges forming the face

    def exited_by(self, p, q):
        """
        Determines whether line segment pq exits the face, i.e., crosses an edge of the face towards the outside.

        :param p: a Point object
        :param q: a Point object
        :returns: the crossed Delaunay edge, or None if pq does not exit the face
        """
        for edge in self.edges:
            if edge.intersects(p, q) and edge.orientation(p) == 2 and edge.orientation(q) == 1:
                return edge

        return None


class Triangle(Face):
    """
    A triangle consisting of exactly three HalfEdge objects.
    """
    def __init__(self):
        super().__init__()

    def get_edge(self, origin, target):
        """
        Returns the edge in the triangle with given origin and target, or None otherwise.
        """
        for edge in self.edges:
            if edge.origin == origin and edge.target == target:
                return edge

        return None

    def __str__(self):
        return f"{self.edges[0].origin} -> {self.edges[0].next.origin} -> {self.edges[0].next.next.origin}"


class OuterFace(Face):
    """
    An outer face consisting of exactly one HalfEdge and two HalfLine objects.
    """
    def __init__(self):
        super().__init__()

    @property
    def half_edge(self):
        """
        Returns the half-edge on the boundary of the face.
        """
        for edge in self.edges:
            if type(edge) == HalfEdge:
                return edge

        raise Exception("Outer face does not contain a HalfEdge")

    def __str__(self):
        return f"<- {self.half_edge} ->"


class DelaunayTriangulation:
    """
    A Delaunay triangulation consisting of a set of Triangle objects and OuterFace objects.
    """
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

        # Construct the Delaunay vertices corresponding to the vertices and obstacles
        dt_vertices = [DelaunayVertex(vertex) for vertex in self.instance.graph.vertices]
        dt_obstacles = [DelaunayVertex(obstacle) for obstacle in self.instance.obstacles]
        self.vertices = dt_vertices + dt_obstacles

        # Compute the Delaunay triangulation on the vertices and obstacles
        dt_input_points = np.array([[float(vertex.point.x), float(vertex.point.y)] for vertex in self.vertices])
        dt = Delaunay(dt_input_points)

        self.triangles = []
        self.outer_faces = []
        boundary_dt_edges = []

        # Iterate over all triangles
        for i in range(len(dt.simplices)):
            triangle = Triangle()

            # Consider each edge of the triangle
            for j in range(3):
                p1 = self.vertices[dt.simplices[i][j]]
                p2 = self.vertices[dt.simplices[i][(j + 1) % 3]]

                # Construct half-edge
                edge = HalfEdge(p1, p2)
                edge.face = triangle
                p1.outgoing_edges.append(edge)

                # Find other triangle adjacent to the edge
                neighbor_index = dt.neighbors[i][(j + 2) % 3]

                # If neighboring triangle was already created, find twin edge
                if -1 < neighbor_index < i:
                    twin_edge = self.triangles[neighbor_index].get_edge(p2, p1)

                    # Set twin
                    edge.twin = twin_edge
                    twin_edge.twin = edge

                # If no neighboring triangle, construct new outer face
                elif neighbor_index == -1:
                    outer_face = OuterFace()

                    # Construct half-edge of outer face
                    twin_edge = HalfEdge(p2, p1)
                    twin_edge.face = outer_face
                    p2.outgoing_edges.append(twin_edge)

                    # Set twin
                    edge.twin = twin_edge
                    twin_edge.twin = edge

                    outer_face.edges.append(twin_edge)
                    self.outer_faces.append(outer_face)
                    boundary_dt_edges.append(twin_edge)

                triangle.edges.append(edge)

            # Set next and prev
            for j in range(3):
                triangle.edges[j].next = triangle.edges[(j + 1) % 3]
                triangle.edges[j].prev = triangle.edges[(j - 1) % 3]

            self.triangles.append(triangle)

        current_edge = boundary_dt_edges[0]

        # Consider the half-edges on the boundary of the triangulation and construct half-lines defining outer faces
        while boundary_dt_edges:
            for next_edge in boundary_dt_edges:
                if next_edge.origin == current_edge.target:
                    # Compute half-line from DelaunayVertex moving 'outwards' of triangulation
                    half_line_origin = current_edge.target
                    bisector = current_edge.bisector(next_edge.twin)
                    other_point_on_half_line = half_line_origin + bisector

                    # Construct half-line
                    half_line = HalfLine(current_edge.target, other_point_on_half_line)
                    half_line.face = current_edge.face
                    half_line_origin.outgoing_edges.append(half_line)

                    # Construct reverse half-line
                    rev_half_line = HalfLine(current_edge.target, other_point_on_half_line, True)
                    rev_half_line.face = next_edge.face

                    # Set twin
                    half_line.twin = rev_half_line
                    rev_half_line.twin = half_line

                    # Set next and prev
                    current_edge.next = half_line
                    half_line.prev = current_edge
                    next_edge.prev = rev_half_line
                    rev_half_line.next = next_edge

                    boundary_dt_edges.remove(next_edge)
                    current_edge = next_edge
                    break

    def get_delaunay_vertex_from_point(self, point):
        """
        Maps a point to its corresponding vertex in the Delaunay triangulation.

        :param point: a Vertex or PointObstacle object
        :returns: the DelaunayVertex object corresponding to the given point
        """
        if type(point) == Vertex:
            return self.vertices[point.id]
        elif type(point) == PointObstacle:
            return self.vertices[len(self.instance.graph.vertices) + point.id]
        else:
            raise Exception(f"Given point {point} is not a Vertex or PointObstacle")

    def __str__(self):
        result = "Triangles:\n"
        for triangle in self.triangles:
            result += f"- {triangle}\n"

        result += "\nOuter faces:\n"
        for face in self.outer_faces:
            result += f"- {face}\n"
        result = result[:-1]

        return result
