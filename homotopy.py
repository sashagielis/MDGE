from collections import deque

from delaunay_triangulation import DelaunayTriangulation
from point import Point
from utils import on_segment, orientation


class CrossingSequence:
    """
    The (reduced or unreduced) sequence of half-edges of the Delaunay triangulation crossed by the edge.
    Only includes the (inner) half-edges that are crossed when exiting their corresponding triangle.
    """
    def __init__(self, edge):
        """
        :param edge: an Edge object
        """
        self.edge = edge
        self.sequence = []

        self.compute_sequence()

    def compute_initial_triangle(self):
        """
        Computes the first triangle of the Delaunay triangulation that the edge moves through.
        Edge links located on the boundary of a triangle are not considered to be moving 'through' the triangle.

        :returns: the first triangle and the index of the first edge link moving through the triangle
        """
        i = 0

        # Find the first edge link that moves through a triangle adjacent to v1
        while i < len(self.edge.path) - 1:
            target = self.edge.path[i + 1]

            # For each triangle adjacent to v1, check if it contains the edge link
            for he in self.edge.v1.outgoing_dt_edges:
                if he.orientation(target) == he.prev.orientation(target) == 2:
                    return he.triangle, i

            i += 1

        return None, i

    def compute_sequence(self):
        """
        Computes the unreduced crossing sequence of the edge.
        """
        # Compute initial triangle and index of first edge link moving 'through' the triangle
        current_triangle, i = self.compute_initial_triangle()

        # Iterate over the edge links and record crossings
        while i < len(self.edge.path) - 1:
            p1 = self.edge.path[i]
            p2 = self.edge.path[i + 1]

            # Compute which half-edge of the current triangle is crossed by the edge link
            crossed_he = current_triangle.exited_by(p1, p2)

            # If no edge was crossed, consider the next edge link
            # Otherwise, repeat the following until the edge link does not cross the current triangle:
            #   (1) add the crossed edge to the crossing sequence
            #   (2) update the current triangle
            #   (3) compute which half-edge of the current triangle is crossed by the edge link
            while crossed_he is not None:
                self.sequence.append(crossed_he)
                current_triangle = crossed_he.twin.triangle
                crossed_he = current_triangle.exited_by(p1, p2)

            i += 1

    def reduce(self):
        """
        Reduces the crossing sequence to its minimum homotopic equivalent.
        """
        # Iteratively remove adjacent pairs of equivalent crossings
        i = 0
        while i < len(self.sequence) - 1:
            if self.sequence[i] == self.sequence[i + 1].twin:
                del self.sequence[i:i+2]

                i = max(0, i - 1)
            else:
                i += 1

        # Remove redundant edge links incident on vertices
        while self.sequence:
            if any(e in self.edge.v1.outgoing_dt_edges for e in [self.sequence[0], self.sequence[0].twin]):
                del self.sequence[0]
            elif any(e in self.edge.v2.outgoing_dt_edges for e in [self.sequence[-1], self.sequence[-1].twin]):
                del self.sequence[-1]
            else:
                break

    def update(self, he):
        """
        Updates the reduced crossing sequence after flipping half-edge he.

        :param he: a HalfEdge object
        """
        def check_quadrilateral():
            """
            Checks whether the edge enters a quadrilateral for which he is the separating edge via the current crossing.
            Updates the crossing sequence of the edge accordingly.
            """
            # Check if he or its twin is the separating half-edge of the quadrilateral entered via the current crossing
            # The separating half-edge is the half-edge that the edge may cross, taking into account the half-edge side
            separating_half_edge = None
            if crossing == he.next.twin or crossing == he.prev.twin:
                separating_half_edge = he
            elif crossing == he.twin.next.twin or crossing == he.twin.prev.twin:
                separating_half_edge = he.twin

            if separating_half_edge is not None:
                # Get the next two crossings of the edge
                crossing2 = self.sequence[i + 1]
                crossing3 = self.sequence[i + 2] if i < len(self.sequence) - 2 else None

                # If the edge crossed the separating half-edge, check if it still does after the flip
                if crossing2 == he or crossing2 == he.twin:
                    # First delete the crossed half-edge and then replace it by the separating half-edge if necessary
                    # We do this because the separating half-edge may be the twin of crossing2
                    del self.sequence[i + 1]

                    # If crossing2 is the last crossing, remove it as it has become connected to v2 after the flip
                    # If the third crossing is in the same triangle as the separating half-edge, also remove it
                    # Otherwise, (re)insert the separating half-edge
                    if crossing3 is not None and crossing3.triangle != separating_half_edge.triangle:
                        self.sequence.insert(i + 1, separating_half_edge)

                # Otherwise, the edge crosses the separating half-edge after the flip
                else:
                    self.sequence.insert(i + 1, separating_half_edge)

        # Iterate over the crossings of the crossing sequence and update them due to flipping he
        i = 0
        while i < len(self.sequence):
            crossing = self.sequence[i]

            # Handle the case where it is the first crossing after leaving vertex v1
            if i == 0:
                # If he is the first crossing, remove it as he has become connected to v1 after the flip
                if crossing == he or crossing == he.twin:
                    del self.sequence[i]

                # If he was one of v1's incident half-edges before the flip, its twin has become the first crossing
                elif crossing == he.next or crossing == he.prev:
                    self.sequence.insert(i, he.twin)
                    i += 1
                elif crossing == he.twin.next or crossing == he.twin.prev:
                    self.sequence.insert(i, he)
                    i += 1

                # Otherwise, check the quadrilateral that the edge enters via the crossing
                else:
                    check_quadrilateral()
                    i += 1

            # Handle the case where it is the last crossing before arriving at vertex v2
            elif i == len(self.sequence) - 1:
                # If he is the last crossing, remove it as he has become connected to v2 after the flip
                if crossing == he or crossing == he.twin:
                    del self.sequence[i]

                # If he was one of v2's incident half-edges before the flip, it has become the last crossing
                elif crossing == he.next.twin or crossing == he.prev.twin:
                    self.sequence.insert(i + 1, he)
                    i += 2
                elif crossing == he.twin.next.twin or crossing == he.twin.prev.twin:
                    self.sequence.insert(i + 1, he.twin)
                    i += 2
                else:
                    i += 1

            # Handle the general case where it is a crossing via which the edge enters a quadrilateral
            else:
                check_quadrilateral()
                i += 1

    def __str__(self):
        i = 1
        result = ""
        for he in self.sequence:
            result += f"{i}. {he}\n"
            i += 1

        if len(self.sequence) > 0:
            result = result[:-1]

        return result


class Funnel:
    """
    A funnel describing the possible space containing the shortest homotopic path of an edge.
    The funnel consists of a tail (polygonal path) from source to a point called the apex, and a fan (simple polygon).
    """
    def __init__(self, edge):
        """
        :param edge: an Edge object
        """
        self.edge = edge

        # Initialize funnel with v1
        self.tail = None
        self.apex = None
        self.fan = None

        self.compute_funnel()

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
            if old_leftmost_point is self.apex:
                self.apex = new_leftmost_point
                self.tail.append(self.apex)

            # If leftmost point is the apex, the leftmost wedge is on the right concave chain of the fan
            if new_leftmost_point is self.apex:
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
            if old_rightmost_point is self.apex:
                self.apex = new_rightmost_point
                self.tail.append(self.apex)

            # If rightmost point is the apex, the rightmost wedge is on the left concave chain of the fan
            if new_rightmost_point is self.apex:
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
        while self.fan[0] is not self.apex and self.fan[-1] is not self.apex:
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

    def compute_funnel(self):
        """
        Computes the funnel of the reduced crossing sequence of the edge through the Delaunay triangulation.
        Includes all collinear (straight) bends in the final funnel.
        """
        sequence = self.edge.crossing_sequence.sequence

        # If the reduced crossing sequence is empty, the shortest path is given by a straight-line edge
        if len(sequence) == 0:
            self.tail = [self.edge.v1, self.edge.v2]
            self.apex = self.edge.v2
            self.fan = deque([self.edge.v2])
            return

        # Add extra half-edge incident on v2 as 'crossing' to sequence to add v2 to the fan at the end
        last_crossing = sequence[-1]
        for he in self.edge.v2.outgoing_dt_edges:
            if he.target is last_crossing.target:
                sequence.append(he)
                break

        # Initialize funnel with v1
        self.tail = [self.edge.v1]
        self.apex = self.edge.v1
        self.fan = deque([self.edge.v1])

        current_crossing = None

        # Consider each crossing in turn
        i = 0
        while i < len(sequence):
            # Consider the next crossing
            next_crossing = sequence[i]

            # If fan consists of only one point, it is the apex
            # This happens when the funnel is not yet initialized, or the edge left the fan via the previous crossing
            # We then rebuild the fan starting from the first crossing not incident on the apex
            # In other words, we build a new funnel as the previous one (if any) cannot be grown any further
            if len(self.fan) == 1:
                if next_crossing.origin is not self.apex and next_crossing.target is not self.apex:
                    self.fan.appendleft(next_crossing.target)
                    self.fan.append(next_crossing.origin)

            # Otherwise, the next crossing differs in exactly one point from the current crossing
            # We then contract the funnel accordingly from the left or right
            else:
                if current_crossing.origin is next_crossing.origin:
                    new_point = next_crossing.target

                    # Contract the funnel from the left inwards
                    self.contract_left(new_point)
                else:
                    new_point = next_crossing.origin

                    # Contract the funnel from the right inwards
                    self.contract_right(new_point)

            # When contracting the funnel wrt a point that is collinear with a fan chain, we simply add it to the chain
            # As a result, the starts of the left and right concave chain of the fan may overlap due to collinear points
            # We can then advance the tail such that the chains of the fan only intersect in the apex
            # By doing this after each contraction, all collinear (straight) bends are included in the final funnel
            self.advance_tail()

            current_crossing = next_crossing
            i += 1

        # If fan consists of only one point, it does not contain v2 as v2 cannot be the apex
        # This is because the last contraction of the funnel was with respect to v2
        # Therefore, we can set v2 as the apex and add it to the funnel
        if len(self.fan) == 1:
            self.apex = self.edge.v2
            self.tail.append(self.edge.v2)

            # Update the fan
            del self.fan[-1]
            self.fan.append(self.edge.v2)

        # Remove the added crossing incident on v2 from the sequence
        del sequence[-1]

    def compute_shortest_path(self):
        """
        Computes the shortest path through the funnel, including all collinear (straight) bends.

        :returns: a list of Point objects describing the shortest homotopic path of the edge
        """
        # Initialize the shortest path as the tail of the funnel
        shortest_path = self.tail

        found_apex = False

        # If v2 is the leftmost point of the fan, the shortest path consists of the tail plus the left concave chain
        if self.fan[0] is self.edge.v2:
            while self.fan:
                point = self.fan.pop()

                if found_apex:
                    shortest_path.append(point)
                elif point is self.apex:
                    found_apex = True

        # Otherwise, the shortest path consists of the tail plus the right concave chain
        else:
            while self.fan:
                point = self.fan.popleft()

                if found_apex:
                    shortest_path.append(point)
                elif point is self.apex:
                    found_apex = True

        return shortest_path

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

            if point is self.apex:
                found_apex = True

            result += " <- " if not found_apex else " -> "

        if len(self.fan) > 0:
            result = result[:-4]
        else:
            result = result[:-1]

        return result


class Homotopy:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

        # Compute Delaunay triangulation on the vertices and obstacles
        # Construct extra points corresponding to a bounding box of the instance
        tl_bbox_point = Point(self.instance.min_x - 1, self.instance.max_y + 1)
        tr_bbox_point = Point(self.instance.max_x + 1, self.instance.max_y + 1)
        br_bbox_point = Point(self.instance.max_x + 1, self.instance.min_y - 1)
        bl_bbox_point = Point(self.instance.min_x - 1, self.instance.min_y - 1)
        self.bbox_points = [tl_bbox_point, tr_bbox_point, br_bbox_point, bl_bbox_point]

        # Compute Delaunay triangulation
        dt_points = self.instance.graph.vertices + self.instance.obstacles + self.bbox_points
        self.dt = DelaunayTriangulation(dt_points)

    def update_bbox_points(self):
        """
        Updates the bounding box points of the Delaunay triangulation using the instance's dimensions.
        """
        self.bbox_points[0].x = self.instance.min_x - 1
        self.bbox_points[0].y = self.instance.max_y + 1
        self.bbox_points[1].x = self.instance.max_x + 1
        self.bbox_points[1].y = self.instance.max_y + 1
        self.bbox_points[2].x = self.instance.max_x + 1
        self.bbox_points[2].y = self.instance.min_y - 1
        self.bbox_points[3].x = self.instance.min_x - 1
        self.bbox_points[3].y = self.instance.min_y - 1

    def compute_shortest_edges(self, use_existing_crossing_sequences=False):
        """
        Computes the shortest homotopic edges.
        Follows algorithm by Hershberger and Snoeyink (https://doi.org/10.1016/0925-7721(94)90010-8).
        More detailed explanation: https://jeffe.cs.illinois.edu/teaching/compgeom/notes/05-shortest-homotopic.pdf.
        """
        for edge in self.instance.graph.edges:
            if not use_existing_crossing_sequences:
                # Compute the crossing sequence of the edge
                sequence = CrossingSequence(edge)

                # Reduce the crossing sequence
                sequence.reduce()

                # Assign the reduced crossing sequence to the edge
                edge.crossing_sequence = sequence

            # Compute the funnel of the reduced crossing sequence
            funnel = Funnel(edge)

            # Compute the shortest path through the funnel
            shortest_path = funnel.compute_shortest_path()

            # Assign the shortest homotopic path to the edge
            edge.path = shortest_path
