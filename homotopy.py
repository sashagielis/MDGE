from collections import deque

from delaunay_triangulation import DelaunayTriangulation
from utils import orientation


class Funnel:
    """
    A funnel describing the possible space containing the shortest homotopic path of an edge.
    The funnel consists of a tail (polygonal path) from source to a point called the apex, and a fan (simple polygon).
    """
    def __init__(self, tail, apex, fan):
        """
        :param tail: a list of Point objects
        :param apex: a Point object
        :param fan: a list of Point objects describing the fan from left to right
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

        # While the newly added point is not to the left of the leftmost wedge, shrink the funnel
        while orientation(wedge_origin, wedge_left_target, point) != 2:
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

        # While the newly added point is not to the right of the rightmost wedge, shrink the funnel
        while orientation(wedge_origin, wedge_right_target, point) != 1:
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

    def __str__(self):
        result = "Tail: "
        for point in self.tail:
            result += str(point) + " -> "

        if len(self.tail) > 0:
            result = result[:-4]
        else:
            result = result[:-1]

        result += "\nFan: "
        for point in self.fan:
            result += str(point) + " -> "

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
    # Retrieve Delaunay vertex corresponding to first edge vertex
    i = 0

    # Find the first edge link that moves through a triangle adjacent to v1
    while i < len(edge.path) - 1:
        target = edge.path[i + 1]

        # For each triangle adjacent to v1, check if it contains the edge link
        for dt_edge in edge.v1.outgoing_edges:
            if dt_edge.orientation(target) == dt_edge.prev.orientation(target) == 2:
                return dt_edge.triangle, i

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
        crossed_dt_edge = current_triangle.exited_by(p1, p2)

        # If no edge was crossed, consider the next edge link
        # Otherwise, repeat the following until the edge link does not cross the current triangle:
        #   (1) add the crossed edge to the crossing sequence
        #   (2) update the current triangle
        #   (3) compute which half-edge of the current triangle is crossed by the edge link
        while crossed_dt_edge is not None:
            sequence.append(crossed_dt_edge)
            current_triangle = crossed_dt_edge.twin.triangle
            crossed_dt_edge = current_triangle.exited_by(p1, p2)

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
            reduced_sequence.pop()
        else:
            reduced_sequence.append(he)

    # Remove redundant edge links incident on vertices
    while reduced_sequence:
        if any(e in edge.v1.outgoing_edges for e in [reduced_sequence[0], reduced_sequence[0].twin]):
            reduced_sequence.pop(0)
        elif any(e in edge.v2.outgoing_edges for e in [reduced_sequence[-1], reduced_sequence[-1].twin]):
            reduced_sequence.pop()
        else:
            break

    return reduced_sequence


def compute_funnel(sequence, edge):
    """
    Computes the funnel of the reduced crossing sequence through the Delaunay triangulation.

    :param sequence: a list describing the reduced crossing sequence of the edge
    :param edge: an Edge object
    :returns the funnel of the edge
    """
    # If the reduced crossing sequence is empty, the shortest path is given by a straight-line edge
    if len(sequence) == 0:
        return Funnel([edge.v1, edge.v2], edge.v2, [])

    # Add extra half-edge incident on v2 as 'crossing' to sequence to add v2 to the fan at the end
    last_crossing = sequence[-1]
    for he in edge.v2.outgoing_edges:
        if he.target == last_crossing.target:
            sequence.append(he)
            break

    current_crossing = sequence[0]

    # Initialize funnel with v1 and first crossed half-edge
    tail = [edge.v1]
    apex = edge.v1
    fan = [current_crossing.target, edge.v1, current_crossing.origin]
    funnel = Funnel(tail, apex, fan)

    i = 1

    # Consider each crossing in turn
    while i < len(sequence):
        next_crossing = sequence[i]

        # If fan consists of only one point, it is the apex
        # This happens when the previous crossing caused the edge to leave the fan
        # We then rebuild the fan starting from the first crossing not incident on the apex
        # In other words, we build a new funnel as the previous one cannot be grown any further
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

        current_crossing = next_crossing
        i += 1

    # If fan consists of only one point, it does not contain v2 as v2 cannot be the apex
    # This is because the last contraction of the funnel was with respect to v2
    # Therefore, we add v2 to the fan
    if len(funnel.fan) == 1:
        funnel.fan.append(edge.v2)

    return funnel


def compute_shortest_path(funnel, edge):
    """
    Computes the shortest path through the funnel.

    :param funnel: a Funnel object corresponding to the edge
    :param edge: an Edge object
    :returns: a list of Point objects describing the shortest homotopic path of the edge
    """
    # Initialize the shortest path as the tail of the funnel
    shortest_path = funnel.tail

    # If the fan is empty, the shortest path is given by the tail
    if len(funnel.fan) == 0:
        return shortest_path

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
        self.dt = DelaunayTriangulation(self.instance)

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

            # Compute the funnel of the reduced crossing sequence
            funnel = compute_funnel(reduced_sequence, edge)

            # Compute the shortest path through the funnel
            shortest_path = compute_shortest_path(funnel, edge)

            # Assign the shortest homotopic path to the edge
            edge.path = shortest_path
