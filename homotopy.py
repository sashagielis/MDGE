from collections import deque

from delaunay_triangulation import DelaunayTriangulation
from point import Point
from utils import orientation, on_segment


class Funnel:
    """
    A funnel describing the possible space containing the shortest homotopic path of an edge.
    The funnel consists of a tail (polygonal path) from source to a point called the apex, and a fan (simple polygon).
    """
    def __init__(self, tail, apex, fan):
        """
        :param tail: a list of Point objects
        :param apex: a Point object
        :param fan: a list of Point objects describing the fan from left to right, which should contain the apex
        """
        self.tail = tail
        self.apex = apex
        self.fan = deque(fan)

    def contract_left(self, point):
        """
        Contracts the funnel from the left inwards based on the newly added point.

        :param point: a Point object
        """
        # Initialize the leftmost wedge
        wedge_origin = self.fan[1]
        wedge_left_target = self.fan[0]

        # While the newly added point is to the right of the leftmost wedge, shrink the funnel
        while orientation(wedge_origin, wedge_left_target, point) == 1:
            # Remove the leftmost point from the fan
            old_leftmost_point = self.fan.popleft()
            new_leftmost_point = self.fan[0]

            # If removed point was the apex, set new apex
            if old_leftmost_point == self.apex:
                self.apex = new_leftmost_point
                self.tail.append(self.apex)

            # If leftmost point is the apex, the leftmost wedge is on the right concave chain of the fan
            if new_leftmost_point == self.apex:
                # If fan consists of only one point, we cannot shrink the funnel any further
                if len(self.fan) == 1:
                    return
                else:
                    wedge_origin = new_leftmost_point
                    wedge_left_target = self.fan[1]

            # Otherwise, the leftmost wedge is on the left concave chain of the fan
            else:
                wedge_origin = self.fan[1]
                wedge_left_target = new_leftmost_point

        # Extend fan to the left
        self.fan.appendleft(point)

    def contract_right(self, point):
        """
        Contracts the funnel from the right inwards based on the newly added point.

        :param point: a Point object
        """
        # Initialize the rightmost wedge
        wedge_origin = self.fan[-2]
        wedge_right_target = self.fan[-1]

        # While the newly added point is to the left of the rightmost wedge, shrink the funnel
        while orientation(wedge_origin, wedge_right_target, point) == 2:
            # Remove the rightmost point from the fan
            old_rightmost_point = self.fan.pop()
            new_rightmost_point = self.fan[-1]

            # If removed point was the apex, set new apex
            if old_rightmost_point == self.apex:
                self.apex = new_rightmost_point
                self.tail.append(self.apex)

            # If rightmost point is the apex, the rightmost wedge is on the left concave chain of the fan
            if new_rightmost_point == self.apex:
                # If fan consists of only one point, we cannot shrink the funnel any further
                if len(self.fan) == 1:
                    return
                else:
                    wedge_origin = new_rightmost_point
                    wedge_right_target = self.fan[-2]

            # Otherwise, the rightmost wedge is on the right concave chain of the fan
            else:
                wedge_origin = self.fan[-2]
                wedge_right_target = new_rightmost_point

        # Extend fan to the right
        self.fan.append(point)

    def advance_tail(self):
        """
        Advances the tail of the funnel until the concave chains of the fan only intersect in the apex.
        """
        # Since the chains of the fan are concave, we can check for chain overlap starting from the apex
        apex_index = self.fan.index(self.apex)
        while self.fan[0] != self.apex and self.fan[-1] != self.apex:
            # Get the next point of the left chain and the next point of the right chain
            p1 = self.fan[apex_index - 1]
            p2 = self.fan[apex_index + 1]

            # If p2 lies on the line segment between the apex and p1, we advance the tail to p2
            if on_segment(self.apex, p1, p2):
                self.apex = p2
                self.tail.append(p2)

                # Update the fan
                del self.fan[apex_index]

            # If p1 lies on the line segment between the apex and p2, we advance the tail to p1
            elif on_segment(self.apex, p2, p1):
                self.apex = p1
                self.tail.append(p1)

                # Update the fan
                del self.fan[apex_index]

                # Since the new apex p1 was located one before the previous apex in the fan, decrease apex index by one
                apex_index -= 1

            # Otherwise, the current concave chains of the fan only intersect in the apex by definition, so we are done
            else:
                break

    def __str__(self):
        result = "Tail: "
        for point in self.tail:
            result += str(point) + " -> "

        if len(self.tail) > 0:
            result = result[:-4]
        else:
            result = result[:-1]

        result += "\nFan: "
        found_apex = False
        for point in self.fan:
            result += str(point)

            if point == self.apex:
                found_apex = True

            result += " <- " if not found_apex else " -> "

        if len(self.fan) > 0:
            result = result[:-4]
        else:
            result = result[:-1]

        return result


def compute_initial_triangle(edge):
    """
    Computes the first triangle of the Delaunay triangulation that the edge moves through.
    Edge links located on the boundary of a triangle are not considered to be moving 'through' the triangle.

    :param edge: an Edge object
    :returns: the first triangle and the index of the first edge link moving through the triangle
    """
    i = 0

    # Find the first edge link that moves through a triangle adjacent to v1
    while i < len(edge.path) - 1:
        target = edge.path[i + 1]

        # For each triangle adjacent to v1, check if it contains the edge link
        for he in edge.v1.outgoing_dt_edges:
            if he.orientation(target) == he.prev.orientation(target) == 2:
                return he.triangle, i

        i += 1

    return None, i


def compute_crossing_sequence(edge):
    """
    Computes the (unreduced) sequence of half-edges of the Delaunay triangulation crossed by the edge.
    Only records the half-edges that are crossed when exiting their corresponding triangle.

    :param edge: an Edge object
    :returns: a list describing the crossing sequence of the edge
    """
    sequence = []

    # Compute initial triangle and index of first edge link moving 'through' the triangle
    current_triangle, i = compute_initial_triangle(edge)

    # Iterate over the edge links and record crossings
    while i < len(edge.path) - 1:
        p1 = edge.path[i]
        p2 = edge.path[i + 1]

        # Compute which half-edge of the current triangle is crossed by the edge link
        crossed_he = current_triangle.exited_by(p1, p2)

        # If no edge was crossed, consider the next edge link
        # Otherwise, repeat the following until the edge link does not cross the current triangle:
        #   (1) add the crossed edge to the crossing sequence
        #   (2) update the current triangle
        #   (3) compute which half-edge of the current triangle is crossed by the edge link
        while crossed_he is not None:
            sequence.append(crossed_he)
            current_triangle = crossed_he.twin.triangle
            crossed_he = current_triangle.exited_by(p1, p2)

        i += 1

    return sequence


def reduce_crossing_sequence(sequence, edge):
    """
    Reduces the crossing sequence of an edge to its minimum homotopic equivalent.

    :param sequence: a list describing the crossing sequence of the edge
    :param edge: an Edge object
    :returns: a list describing the reduced crossing sequence of the edge
    """
    reduced_sequence = []

    # Iteratively remove adjacent pairs of equivalent crossings
    for he in sequence:
        if reduced_sequence and reduced_sequence[-1] == he.twin:
            del reduced_sequence[-1]
        else:
            reduced_sequence.append(he)

    # Remove redundant edge links incident on vertices
    while reduced_sequence:
        if any(e in edge.v1.outgoing_dt_edges for e in [reduced_sequence[0], reduced_sequence[0].twin]):
            del reduced_sequence[0]
        elif any(e in edge.v2.outgoing_dt_edges for e in [reduced_sequence[-1], reduced_sequence[-1].twin]):
            del reduced_sequence[-1]
        else:
            break

    return reduced_sequence


def compute_funnel(sequence, edge):
    """
    Computes the funnel of the reduced crossing sequence through the Delaunay triangulation.
    Includes all collinear (straight) bends in the final funnel.

    :param sequence: a list describing the reduced crossing sequence of the edge
    :param edge: an Edge object
    :returns the funnel of the edge
    """
    # If the reduced crossing sequence is empty, the shortest path is given by a straight-line edge
    if len(sequence) == 0:
        return Funnel([edge.v1, edge.v2], edge.v2, [edge.v2])

    # Add extra half-edge incident on v2 as 'crossing' to sequence to add v2 to the fan at the end
    last_crossing = sequence[-1]
    for he in edge.v2.outgoing_dt_edges:
        if he.target == last_crossing.target:
            sequence.append(he)
            break

    current_crossing = None

    # Initialize funnel with v1
    tail = [edge.v1]
    apex = edge.v1
    fan = [edge.v1]
    funnel = Funnel(tail, apex, fan)

    # Consider each crossing in turn
    i = 0
    while i < len(sequence):
        # Consider the next crossing
        next_crossing = sequence[i]

        # If fan consists of only one point, it is the apex
        # This happens when the funnel is not yet initialized, or the previous crossing caused the edge to leave the fan
        # We then rebuild the fan starting from the first crossing not incident on the apex
        # In other words, we build a new funnel as the previous one (if any) cannot be grown any further
        if len(funnel.fan) == 1:
            if next_crossing.origin != funnel.apex and next_crossing.target != funnel.apex:
                funnel.fan.appendleft(next_crossing.target)
                funnel.fan.append(next_crossing.origin)

        # Otherwise, the next crossing differs in exactly one point from the current crossing
        # We then contract the funnel accordingly from the left or right
        else:
            if current_crossing.origin == next_crossing.origin:
                new_point = next_crossing.target

                # Contract the funnel from the left inwards
                funnel.contract_left(new_point)
            else:
                new_point = next_crossing.origin

                # Contract the funnel from the right inwards
                funnel.contract_right(new_point)

        # When contracting the funnel wrt a point that is collinear with a fan chain, we simply add it to the chain
        # As a result, the starts of the left and right concave chain of the fan may overlap due to collinear points
        # We can then advance the tail such that the chains of the fan only intersect in the apex
        # By doing this after each contraction, all collinear (straight) bends are included in the final funnel
        funnel.advance_tail()

        current_crossing = next_crossing
        i += 1

    # If fan consists of only one point, it does not contain v2 as v2 cannot be the apex
    # This is because the last contraction of the funnel was with respect to v2
    # Therefore, we can set v2 as the apex and add it to the funnel
    if len(funnel.fan) == 1:
        funnel.apex = edge.v2
        funnel.tail.append(edge.v2)

        # Update the fan
        del funnel.fan[-1]
        funnel.fan.append(edge.v2)

    return funnel


def compute_shortest_path(funnel, edge):
    """
    Computes the shortest path through the funnel, including all collinear (straight) bends.

    :param funnel: a Funnel object corresponding to the edge
    :param edge: an Edge object
    :returns: a list of Point objects describing the shortest homotopic path of the edge
    """
    # Initialize the shortest path as the tail of the funnel
    shortest_path = funnel.tail

    found_apex = False

    # If v2 is the leftmost point of the fan, the shortest path consists of the tail plus the left concave chain
    if funnel.fan[0] == edge.v2:
        while funnel.fan:
            point = funnel.fan.pop()

            if found_apex:
                shortest_path.append(point)
            elif point == funnel.apex:
                found_apex = True

    # Otherwise, the shortest path consists of the tail plus the right concave chain
    else:
        while funnel.fan:
            point = funnel.fan.popleft()

            if found_apex:
                shortest_path.append(point)
            elif point == funnel.apex:
                found_apex = True

    return shortest_path


class Homotopy:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

        # Compute Delaunay triangulation on the vertices and obstacles
        self.dt = self.compute_delaunay_triangulation()

    def compute_delaunay_triangulation(self):
        """
        Computes a Delaunay triangulation on the vertices and obstacles in the instance.
        Four extra points are added to specify a bounding box of the instance.

        :returns: a DelaunayTriangulation object
        """
        # Construct extra Delaunay points corresponding to a bounding box of the instance
        # This makes it easier to compute the shortest homotopic edges using the funnel algorithm
        tl_bbox_point = Point(self.instance.min_x - 1, self.instance.max_y + 1)
        tr_bbox_point = Point(self.instance.max_x + 1, self.instance.max_y + 1)
        br_bbox_point = Point(self.instance.max_x + 1, self.instance.min_y - 1)
        bl_bbox_point = Point(self.instance.min_x - 1, self.instance.min_y - 1)
        extra_dt_points = [tl_bbox_point, tr_bbox_point, br_bbox_point, bl_bbox_point]

        dt_points = self.instance.graph.vertices + self.instance.obstacles + extra_dt_points

        # Compute Delaunay triangulation
        dt = DelaunayTriangulation(dt_points)

        return dt

    def compute_shortest_edges(self):
        """
        Computes the shortest homotopic edges.
        Follows algorithm by Hershberger and Snoeyink (https://doi.org/10.1016/0925-7721(94)90010-8).
        More detailed explanation: https://jeffe.cs.illinois.edu/teaching/compgeom/notes/05-shortest-homotopic.pdf.
        """
        for edge in self.instance.graph.edges:
            # Compute the crossing sequence of the edge
            sequence = compute_crossing_sequence(edge)

            # Reduce the crossing sequence
            reduced_sequence = reduce_crossing_sequence(sequence, edge)

            # Assign the reduced crossing sequence to the edge
            edge.crossing_sequence = reduced_sequence

            # Compute the funnel of the reduced crossing sequence
            funnel = compute_funnel(reduced_sequence, edge)

            # Compute the shortest path through the funnel
            shortest_path = compute_shortest_path(funnel, edge)

            # Assign the shortest homotopic path to the edge
            edge.path = shortest_path
