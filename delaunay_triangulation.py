import numpy as np
from scipy.spatial import Delaunay

from point import Point
from utils import check_segment_segment_intersection, orientation


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
        :returns: the crossed half-edge, or None if pq does not exit the triangle
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
    A Delaunay triangulation on the vertices and obstacles in the instance.
    Extra points are added to specify small bounding boxes for the vertices and obstacles to deal with homotopy.
    Another four extra points are added to specify a bounding box of the instance.
    """
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

        extra_dt_points = []

        # Construct extra Delaunay points corresponding to a small bounding box for each vertex and point obstacle
        # This helps to identify along which sides of the obstacles the shortest homotopic edges move
        # Delta must be small enough to prevent edges from moving through the bounding box (except for their endpoints')
        # At the same time, SciPy Delaunay cannot handle too small coordinate differences (0.00001 does not work)
        delta = 0.0001
        dt_points = self.instance.graph.vertices + self.instance.obstacles
        for point in dt_points:
            # Reset outgoing edges
            point.outgoing_edges = []

            tl_box_point = point + Point(-delta, delta)
            tr_box_point = point + Point(delta, delta)
            br_box_point = point + Point(delta, -delta)
            bl_box_point = point + Point(-delta, -delta)

            extra_dt_points.extend([tl_box_point, tr_box_point, br_box_point, bl_box_point])

        # Construct extra Delaunay points corresponding to a bounding box of the instance
        # This makes it easier to compute the shortest homotopic edges using the funnel algorithm
        tl_bbox_point = Point(self.instance.min_x - 1, self.instance.max_y + 1)
        tr_bbox_point = Point(self.instance.max_x + 1, self.instance.max_y + 1)
        br_bbox_point = Point(self.instance.max_x + 1, self.instance.min_y - 1)
        bl_bbox_point = Point(self.instance.min_x - 1, self.instance.min_y - 1)
        extra_dt_points.extend([tl_bbox_point, tr_bbox_point, br_bbox_point, bl_bbox_point])

        self.points = dt_points + extra_dt_points

        # Compute the Delaunay triangulation
        dt_input_points = np.array([[float(point.x), float(point.y)] for point in self.points])
        self.dt = Delaunay(dt_input_points)

        self.triangles = []

        # Iterate over all triangles
        for i in range(len(self.dt.simplices)):
            triangle = Triangle()

            # Consider each edge of the triangle
            for j in range(3):
                p1 = self.points[self.dt.simplices[i][j]]
                p2 = self.points[self.dt.simplices[i][(j + 1) % 3]]

                # Construct half-edge
                he = HalfEdge(p1, p2)
                he.triangle = triangle
                p1.outgoing_edges.append(he)

                # Find other triangle adjacent to the edge
                neighbor_index = self.dt.neighbors[i][(j + 2) % 3]

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

    def __str__(self):
        result = "Triangles:\n"
        for triangle in self.triangles:
            result += f"- {triangle}\n"

        result = result[:-1]

        return result
