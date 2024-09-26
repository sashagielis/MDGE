import numpy as np
from scipy.spatial import Delaunay

from utils import check_segment_segment_intersection, orientation, get_circle, in_circle, on_segment


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

        self.constraint = None  # The minimum-separation constraint on the two points

    def orientation(self, p):
        """
        Determines the orientation of point p with respect to the half-edge.

        :param p: a Point object
        :returns: 0 : collinear, 1 : clockwise, 2 : counterclockwise
        """
        return orientation(self.origin, self.target, p)

    def intersects(self, p, q):
        """
        Determines whether the half-edge intersects line segment pq.

        :param p: a Point object
        :param q: a Point object
        """
        return check_segment_segment_intersection(self.origin, self.target, p, q)

    def flip(self):
        """
        Flips the half-edge and its twin.
        """
        t1 = self.triangle
        t2 = self.twin.triangle

        # Get the two opposing points of the triangles
        point_t1 = self.next.target
        point_t2 = self.twin.next.target

        # Update outgoing half-edges
        self.origin.outgoing_dt_edges.remove(self)
        self.target.outgoing_dt_edges.remove(self.twin)
        point_t1.outgoing_dt_edges.append(self.twin)
        point_t2.outgoing_dt_edges.append(self)

        # Set new origin and target
        self.origin = point_t2
        self.target = point_t1
        self.twin.origin = point_t1
        self.twin.target = point_t2

        # Set new next and prev
        he_next = self.next
        self.next = self.prev
        self.prev = self.twin.next
        self.twin.next = self.twin.prev
        self.twin.prev = he_next

        # Set new next and prev for other half-edges of t1
        self.next.next = self.prev
        self.next.prev = self
        self.prev.next = self
        self.prev.prev = self.next

        # Set new next and prev for other half-edges of t2
        self.twin.next.next = self.twin.prev
        self.twin.next.prev = self.twin
        self.twin.prev.next = self.twin
        self.twin.prev.prev = self.twin.next

        # Set new triangle for other half-edges
        self.next.triangle = t1
        self.prev.triangle = t1
        self.twin.next.triangle = t2
        self.twin.prev.triangle = t2

        # Set new half-edges of t1 and t2
        t1.half_edges = [self, self.next, self.prev]
        t2.half_edges = [self.twin, self.twin.next, self.twin.prev]

        # Reset the minimum-separation constraint on the flipped half-edge and its twin
        self.constraint = None
        self.twin.constraint = None

    def __eq__(self, other):
        if self is None or other is None:
            return False

        return self.origin == other.origin and self.target == other.target

    def __str__(self):
        return f"{self.origin} -> {self.target}"


class Triangle:
    """
    A triangle of the Delaunay triangulation.
    """
    def __init__(self):
        self.half_edges = []  # Set of half-edges forming the triangle

    def get_points(self):
        """
        Returns the three points of the triangle.

        :returns: three Point objects
        """
        if len(self.half_edges) != 3:
            raise Exception(f"Triangle {self} contains {len(self.half_edges)} half-edges instead of 3")

        he = self.half_edges[0]
        p = he.origin
        q = he.target
        r = he.next.target

        return p, q, r

    def get_edge(self, origin, target):
        """
        Returns the half-edge of the triangle with given origin and target.

        :param origin: a Point object
        :param target: a Point object
        :returns: the half-edge from origin to target, or None if it is not part of the triangle
        """
        for he in self.half_edges:
            if he.origin is origin and he.target is target:
                return he

        return None

    def exited_by(self, p, q):
        """
        Determines whether directed line segment pq exits the triangle.
        Checks if pq crosses one of the triangles' half-edges towards the outside.

        :param p: a Point object
        :param q: a Point object
        :returns: the crossed half-edge (either if crossed in a vertex), or None if pq does not exit the triangle
        """
        for he in self.half_edges:
            if (he.intersects(p, q)
                    and (he.orientation(p) == 2 != he.orientation(q)
                         or (he.orientation(p) == 0 and he.orientation(q) == 1))):
                return he

        return None

    def is_delaunay(self):
        """
        Determines whether the triangle satisfies the Delaunay condition.

        :returns: a tuple consisting of a boolean value and the separating half_edge that should be flipped
        """
        p, q, r = self.get_points()

        # If the points are collinear, the triangle is not Delaunay
        if orientation(p, q, r) == 0:
            # The half-edge between the two most distant points should be flipped
            for he in self.half_edges:
                other_point = he.next.target
                if on_segment(he.origin, he.target, other_point):
                    return False, he

        # Get the center and radius of the circumcircle
        center, radius = get_circle(p, q, r)

        # Check for each neighboring triangle if the point not part of the separating edge is inside the circumcircle
        for he in self.half_edges:
            if he.twin is not None:
                opposing_point = he.twin.next.target
                if in_circle(opposing_point, center, radius):
                    return False, he

        return True, None

    def __str__(self):
        p, q, r = self.get_points()

        return f"{p} -> {q} -> {r}"


class DelaunayTriangulation:
    """
    A Delaunay triangulation on a set of points.
    """
    def __init__(self, points):
        """
        :param points: a list of Point objects
        """
        self.vertices = []
        dt_input_points = []

        # Initialize Delaunay points
        for point in points:
            point.outgoing_dt_edges = []
            dt_input_points.append([float(point.x), float(point.y)])
            self.vertices.append(point)

        self.half_edges = []
        self.triangles = []

        # Compute Delaunay triangulation
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
                self.half_edges.append(he)
                he.triangle = triangle
                p1.outgoing_dt_edges.append(he)

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

    def is_valid(self):
        """
        Determines whether all triangles satisfy the Delaunay condition.
        """
        return all(t.is_delaunay()[0] for t in self.triangles)

    def __str__(self):
        result = "Triangles:\n"
        for triangle in self.triangles:
            result += f"- {triangle}\n"

        result = result[:-1]

        return result
