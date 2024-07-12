import copy
import math

from fractions import Fraction

from point import Point
from utils import angle, check_rectangle_arc_intersection, distance, normalize_angle, on_segment, orientation


class StraightBundle:
    """
    A set of straights associated with the same two points.
    """
    def __init__(self):
        # Basic attributes
        self.left = None  # The outermost adjacent elbow bundle on the left
        self.right = None  # The outermost adjacent elbow bundle on the right
        self.size = None  # The number of segments contained in the bundle
        self.thickness = None  # The total thickness of the segments in the bundle

        # Additional attributes
        self.is_terminal = False  # Whether it is a terminal straight bundle (i.e. connected to a terminal elbow bundle)

    def is_associated_with(self, p):
        """
        Determines whether the straight bundle is associated with the given point.

        :param p: a Point object
        """
        return self.left.point is p or self.right.point is p

    def is_connected_to(self, eb):
        """
        Determines whether the straight bundle is connected to the given elbow bundle.

        :param eb: an ElbowBundle object
        """
        return self.left == eb or self.right == eb

    def next(self, prev_eb):
        """
        Returns the next elbow bundle connected to the straight bundle following the given previous elbow bundle.

        :param prev_eb: an ElbowBundle object
        """
        if not self.is_connected_to(prev_eb):
            raise Exception(f"Straight bundle {self} is not connected to elbow bundle {prev_eb}")

        return self.right if self.left == prev_eb else self.left

    def is_closer_than(self, sb, p):
        """
        Determines whether the straight bundle is closer to the given point p than the given straight bundle sb.
        The straight bundles must both be associated with p.

        :param sb: a StraightBundle object
        :param p: a Point object
        """
        if not self.is_associated_with(p):
            raise Exception(f"Straight bundle {self} is not associated with point {p}")

        if not sb.is_associated_with(p):
            raise Exception(f"Straight bundle {sb} is not associated with point {p}")

        if self.left.point is p:
            eb1 = self.left
        else:
            eb1 = self.right

        if sb.left.point is p:
            eb2 = sb.left
        else:
            eb2 = sb.right

        return eb1.is_closer_than(eb2)

    def has_same_orientation_as(self, sb):
        """
        Determines whether the straight bundle has the same orientation as the given straight bundle sb.
        The straight bundles must be associated with the same two points.

        :param sb: a StraightBundle object
        """
        if not self.is_associated_with(sb.left.point) or not self.is_associated_with(sb.right.point):
            raise Exception(f"Straight bundles {self} and {sb} are associated with different points")

        return self.left.point is sb.left.point

    def get_angle(self, t):
        """
        Returns the angle of the straight bundle in radians at the given time t, which is the angle of backbone p'q'.
        Computes the angle by rotating line segment pq based on offsets d(p, p') and d(q, q') and elbow orientations.
        The auxiliary file 'Computing angles of bundles.png' provides a more detailed explanation.

        :param t: the time between 0 and 1
        """
        # Get the left and right endpoint p and q of the backbone of the thin straight bundle
        eb_left = self.left
        eb_right = self.right
        p = eb_left.point
        q = eb_right.point

        # Compute the angle alpha of line segment pq
        alpha = Fraction(angle(p, q))

        # If eb_left is a terminal elbow, p' = p is the left point of the straight's backbone with offset a = 0
        if eb_left.is_terminal:
            a = 0
        # Otherwise, eb_left bends around p, and we compute the offset a = d(p, p') based on the separating thickness
        else:
            a = t * (eb_left.layer_thickness + eb_left.thickness / 2)

        # If eb_right is a terminal elbow, q' = q is the right point of the straight's backbone with offset b = 0
        if eb_right.is_terminal:
            b = 0
        # Otherwise, eb_right bends around q, and we compute the offset b = d(q, q') based on the separating thickness
        else:
            b = t * (eb_right.layer_thickness + eb_right.thickness / 2)

        d_pq = distance(p, q)

        # Compute the angle of rotation beta between line segments pq and p'q' using a and b
        # If the elbow bundles have the same orientation, they are on the same 'side' of the straight bundle
        if not (eb_left.right == self) ^ (eb_right.left == self):
            beta = Fraction(math.asin(abs(b - a) / d_pq))
        # Otherwise, the elbow bundles are on different sides of the straight bundle
        else:
            beta = Fraction(math.asin((a + b) / d_pq))

        # Compute the angle of line segment p'q' using alpha and beta
        # Case 1: the elbow bundles have the same orientation as the straight bundle
        if eb_left.right == self and eb_right.left == self:
            # If a < b, the rotation is counterclockwise
            if a < b:
                ang = alpha + beta
            # Otherwise, the rotation is clockwise
            else:
                ang = alpha - beta
        # Case 2: the elbow bundles have the same orientation but different from the straight bundle
        elif eb_left.left == self and eb_right.right == self:
            # If a < b, the rotation is clockwise
            if a < b:
                ang = alpha - beta
            # Otherwise, the rotation is counterclockwise
            else:
                ang = alpha + beta
        # Case 3: the elbow bundles have different orientations but eb_left has the same as the straight bundle
        # Then, the rotation is clockwise
        elif eb_left.right == self and eb_right.right == self:
            ang = alpha - beta
        # Case 4: the elbow bundles have different orientations but eb_right has the same as the straight bundle
        # Then, the rotation is counterclockwise
        else:
            ang = alpha + beta

        # Normalize the angle
        ang = normalize_angle(ang)

        return ang

    def get_backbone_endpoints(self, t):
        """
        Returns the two endpoints p1 and p2 of the backbone of the straight bundle at the given time t.

        :param t: the time between 0 and 1
        :returns: p1 and p2, ordered from left to right
        """
        eb_left = self.left
        eb_right = self.right
        p1 = eb_left.point
        p2 = eb_right.point

        # Get the angle of the straight bundle at time t
        sb_angle = self.get_angle(t)

        # The angles of the backbone endpoints wrt their elbow bundles are perpendicular to sb_angle
        rotation = Fraction(math.pi / 2)

        # If eb_left is a terminal elbow, p1 is the left point of the backbone
        # Otherwise, eb_left bends around p1, and we translate p1 based on sb_angle and separating thickness
        if not eb_left.is_terminal:
            if eb_left.right == self:
                eb_left_angle = sb_angle + rotation
            else:
                eb_left_angle = sb_angle - rotation

            direction = Point(math.cos(eb_left_angle), math.sin(eb_left_angle))
            magnitude = t * (eb_left.layer_thickness + self.thickness / 2)
            p1 += direction * magnitude

        # If eb_right is a terminal elbow, p2 is the right point of the backbone
        # Otherwise, eb_right bends around p2, and we translate p2 based on sb_angle and separating thickness
        if not eb_right.is_terminal:
            if eb_right.left == self:
                eb_right_angle = sb_angle + rotation
            else:
                eb_right_angle = sb_angle - rotation

            direction = Point(math.cos(eb_right_angle), math.sin(eb_right_angle))
            magnitude = t * (eb_right.layer_thickness + self.thickness / 2)
            p2 += direction * magnitude

        return p1, p2

    def get_corners(self, t):
        """
        Returns the four corners of the straight bundle at the given time t.
        Source: https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point
        -and-rotation

        :param t: the time between 0 and 1
        :returns: the four corner points, ordered top right, top left, bottom left, bottom right
        """
        p1, p2 = self.get_backbone_endpoints(t)

        length = distance(p1, p2)
        theta = angle(p1, p2)
        center = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)

        thickness = t * self.thickness

        tr_x = center.x + length / 2 * math.cos(theta) - thickness / 2 * math.sin(theta)
        tr_y = center.y + length / 2 * math.sin(theta) + thickness / 2 * math.cos(theta)

        tl_x = center.x - length / 2 * math.cos(theta) - thickness / 2 * math.sin(theta)
        tl_y = center.y - length / 2 * math.sin(theta) + thickness / 2 * math.cos(theta)

        bl_x = center.x - length / 2 * math.cos(theta) + thickness / 2 * math.sin(theta)
        bl_y = center.y - length / 2 * math.sin(theta) - thickness / 2 * math.cos(theta)

        br_x = center.x + length / 2 * math.cos(theta) + thickness / 2 * math.sin(theta)
        br_y = center.y + length / 2 * math.sin(theta) - thickness / 2 * math.cos(theta)

        return Point(tr_x, tr_y), Point(tl_x, tl_y), Point(bl_x, bl_y), Point(br_x, br_y)

    def splits(self, eb, t):
        """
        Determines whether the straight bundle gets split by the given elbow bundle at the given time t.

        :param eb: an ElbowBundle object
        :param t: the time between 0 and 1
        """
        return eb.splits(self, t)

    def __str__(self):
        return f"{self.left} -> {self.right}"


class ElbowBundle:
    """
    A set of elbows associated with the same point and sharing the same straight bundle on both ends.
    A terminal elbow forms its own bundle and is not merged with other bundles.
    """
    def __init__(self):
        # Basic attributes
        self.point = None  # The associated point
        self.left = None  # The adjacent straight bundle on the left (left = right for terminal elbows)
        self.right = None  # The adjacent straight bundle on the right (left = right for terminal elbows)
        self.inner = None  # The elbow bundle just inside it (None for terminal elbows)
        self.size = None  # The number of segments contained in the bundle (1 for terminal elbows)
        self.thickness = None  # The total thickness of the segments in the bundle (0 for obstacle elbows)
        self.layer_thickness = None  # The total thickness of the paths between eb and eb.point (0 for terminal elbows)

        # Additional attributes
        self.is_terminal = False  # Whether it is a terminal elbow bundle (i.e., associated with a vertex or obstacle)
        self.orientation = 0  # The orientation of the elbows (0 : terminal, 1 : clockwise, 2 : counterclockwise)

    def is_connected_to(self, sb):
        """
        Determines whether the elbow bundle is connected to the given straight bundle.

        :param sb: a StraightBundle object
        """
        return self.left == sb or self.right == sb

    def next(self, prev_sb):
        """
        Returns the next straight bundle connected to the elbow bundle following the given previous straight bundle.

        :param prev_sb: a StraightBundle object
        """
        if not self.is_connected_to(prev_sb):
            raise Exception(f"Elbow bundle {self} is not connected to straight bundle {prev_sb}")

        return self.right if self.left == prev_sb else self.left

    def is_closer_than(self, eb):
        """
        Determines whether the elbow bundle is closer to its associated point than the given elbow bundle eb.
        The elbow bundles must be associated with the same point.
        Assumes that if self.inner == eb.inner == None, the inner field is not yet initialized.

        :param eb: an ElbowBundle object
        """
        if self.point is not eb.point:
            raise Exception(f"Elbow bundles are associated with different points: {self.point} and {eb.point}")

        # Check if one of them is a terminal elbow bundle
        if self.is_terminal:
            return True
        elif eb.is_terminal:
            return False

        # Check if eb is inside self
        inner_self = self.inner
        while inner_self is not None:
            if inner_self == eb:
                return False
            else:
                inner_self = inner_self.inner

        # Check if self is inside eb
        inner_eb = eb.inner
        while inner_eb is not None:
            if inner_eb == self:
                return True
            else:
                inner_eb = inner_eb.inner

        # If we arrive here, we have that self.inner = eb.inner = None
        # Then starting from self and eb, we pairwise move over their elbow bundles in the same direction
        # Since self and eb bend around the same point, we can consider both elbow bundles as a right bend
        # If the orientation of one of the elbow bundles denotes a left bend, we revert the iterating process
        revert_self = self.orientation == 2
        revert_eb = eb.orientation == 2

        # Iterate pairwise over the next elbow bundles of self and eb until they are associated with different points
        # If we arrive at a terminal elbow, the left = right property ensures that the current elbow stays the same
        # Since we assume a maximum degree of one, self and eb cannot arrive at the same terminal elbow
        # Hence, if one arrives at a terminal elbow, the other must lead to an elbow bundle around a different point
        current_self = self
        current_eb = eb
        while current_self.point is current_eb.point:
            current_self = current_self.right.right if not revert_self else current_self.left.left
            current_eb = current_eb.right.right if not revert_eb else current_eb.left.left

        # Determine the elbow bundles just before the current ones
        prev_self = current_self.left.left if not revert_self else current_self.right.right
        prev_eb = current_eb.left.left if not revert_eb else current_eb.right.right

        # If current_eb ended at a terminal elbow, self is closest if its last bend was a left turn
        if prev_self.point is current_eb.point:
            return prev_self.orientation == 2 if not revert_self else prev_self.orientation == 1

        # If current_self ended at a terminal elbow, self is closest if eb's last bend was a right turn
        elif prev_eb.point is current_self.point:
            return prev_eb.orientation == 1 if not revert_eb else prev_eb.orientation == 2

        # Otherwise, current_self and current_eb ended at different non-terminal elbows and prev_self = prev_eb
        else:
            # Handle the case where current_self, current_eb and the last straight are collinear
            if orientation(prev_self.point, current_self.point, current_eb.point) == 0:
                # If the last bend was a right turn, self is closest if current_self is closer to prev_self
                if (not revert_self and prev_self.orientation == 1) or (revert_self and prev_self.orientation == 2):
                    return on_segment(prev_self.point, current_eb.point, current_self.point)

                # If the last bend was a left turn, self is closest if current_eb is closer to prev_self
                else:
                    return on_segment(prev_self.point, current_self.point, current_eb.point)

            # Otherwise, self is closest if its current point is right of eb's last straight
            else:
                return orientation(prev_self.point, current_eb.point, current_self.point) == 1

    def get_angles(self, t):
        """
        Returns the two angles a1 and a2 of the annular wedge in radians at the given time t, or None if it is terminal.

        :param t: the time between 0 and 1
        :returns: a1 and a2, ordered from left to right
        """
        sb_left = self.left
        sb_right = self.right

        # If self is a terminal elbow bundle around an obstacle, represent the point by two equal angles of 0
        if self.is_terminal and sb_left is None:
            return 0, 0

        # The angles of the annular wedge are perpendicular to the angles of the adjacent straight bundles
        rotation = Fraction(math.pi / 2)

        # Compute the left angle by rotating the angle of the left straight bundle
        sb_left_angle = sb_left.get_angle(t)
        if sb_left.right.point is self.point:
            a1 = sb_left_angle + rotation
        else:
            a1 = sb_left_angle - rotation

        # Compute the right angle by rotating the angle of the right straight bundle
        sb_right_angle = sb_right.get_angle(t)
        if sb_right.left.point is self.point:
            a2 = sb_right_angle + rotation
        else:
            a2 = sb_right_angle - rotation

        # Normalize the angles
        a1 = normalize_angle(a1)
        a2 = normalize_angle(a2)

        return a1, a2

    def splits(self, sb, t):
        """
        Determines whether the elbow bundle splits the given straight bundle at the given time t.

        :param sb: a StraightBundle object
        :param t: the time between 0 and 1
        """
        if sb.is_associated_with(self.point):
            return False

        # Check whether the rectangle of the straight bundle intersects the outer arc of the elbow bundle
        # If a straight only touches an elbow, a split event would immediately trigger a merge event
        # This could lead to an indefinite sequence of alternating split and merge events
        # Therefore, we only register proper intersections as split events
        p1, p2, p3, p4 = sb.get_corners(t)
        center = self.point
        thickness = self.thickness / 2 if self.is_terminal else self.thickness
        radius = t * (self.layer_thickness + thickness)
        left_angle, right_angle = self.get_angles(t)

        return check_rectangle_arc_intersection(p1, p2, p3, p4, center, radius, left_angle, right_angle, True)

    def merges(self, t):
        """
        Determines whether the elbow bundle merges with its adjacent straight bundles at the given time t.

        :param t: the time between 0 and 1
        """
        if self.is_terminal:
            return False

        sb_left = self.left
        sb_right = self.right

        # Check whether the elbow bundle is no longer a right bend using the backbones of the adjacent straight bundles
        p1, p2 = sb_left.get_backbone_endpoints(t)
        p3, p4 = sb_right.get_backbone_endpoints(t)

        # Set a and b equal to the endpoints of the backbone of sb_left such that a -> b is directed towards the elbow
        if sb_left.right.point == self.point:
            a = p1
            b = p2
        else:
            a = p2
            b = p1

        # Set c equal to the endpoint of the backbone of sb_right that is furthest from the elbow
        if sb_right.left.point == self.point:
            c = p4
        else:
            c = p3

        return orientation(a, b, c) != 1

    def __str__(self):
        return f"{self.point}"


class CompactRoutingStructure:
    """
    A compact routing structure storing and maintaining a set of thick edges using straight and elbow bundles.
    Follows the implementation by Duncan et al. (https://doi.org/10.1142/S0129054106004315).
    """
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance
        self.straight_bundles = []
        self.elbow_bundles = []

        self.initialize_bundles()

    def initialize_bundles(self):
        """
        Initializes the bundles in the compact routing structure.
        """
        # Construct singular straight and elbow bundles for edges
        for edge in self.instance.graph.edges:
            # Get reduced crossing sequence of the edge to determine the orientations of the bends
            crossing_sequence = edge.crossing_sequence.sequence
            crossing_index = 0

            bend_orientations = []
            i = 1
            while i < len(edge.path) - 1:
                new_point = edge.path[i]
                new_orientation = None

                # Iterate over crossings in crossing sequence until first crossing with bend as endpoint is found
                # The crossings around a bend all have the corresponding point on the same side of the half-edge
                # Therefore, the bend orientation results from whether the point is origin/target of the crossing
                found_orientation = False
                while not found_orientation:
                    crossing = crossing_sequence[crossing_index]

                    # If point is the origin of the crossing, the bend is a right turn
                    if crossing.origin is new_point:
                        new_orientation = 1
                        found_orientation = True

                    # If point is the target of the crossing, the bend is a left turn
                    elif crossing.target is new_point:
                        new_orientation = 2
                        found_orientation = True

                    # Otherwise, check next crossing
                    else:
                        crossing_index += 1

                # Check if the addition of the new bend makes the last bend unnecessary
                if len(bend_orientations) >= 2:
                    p1 = edge.path[i - 2]
                    p2 = edge.path[i - 1]
                    o_p1 = bend_orientations[-2]
                    o_p2 = bend_orientations[-1]

                    # If p2 lies on the line segment between p1 and the new bend, and the bends have the same
                    # orientation, we can remove p2 from the path as it does not add any path information
                    # Leaving p2 on the path would lead to creating an unnecessary and unwanted straight bend
                    if on_segment(p1, new_point, p2) and o_p1 == o_p2 == new_orientation:
                        # Remove p2 from the path
                        del edge.path[i - 1]
                        del bend_orientations[-1]

                        # Add orientation of new bend
                        # Do not increment i since we removed a previous point from the path
                        bend_orientations.append(new_orientation)
                    else:
                        # Add orientation of new bend
                        bend_orientations.append(new_orientation)
                        i += 1
                else:
                    # Add orientation of new bend
                    bend_orientations.append(new_orientation)
                    i += 1

            straight_bundles = []
            elbow_bundles = []
            for i in range(len(edge.path) - 1):
                p1 = edge.path[i]
                p2 = edge.path[i + 1]

                # Construct straight bundle
                sb = StraightBundle()
                straight_bundles.append(sb)

                # If p1 = v1 is the first point, we construct a terminal elbow bundle around v1
                if i == 0:
                    eb1 = ElbowBundle()
                    elbow_bundles.append(eb1)
                    p1.elbow_bundles.append(eb1)

                    # Set eb1 properties
                    eb1.point = p1
                    eb1.left = sb
                    eb1.size = 1
                    eb1.thickness = edge.thickness

                    # Set is_terminal
                    eb1.is_terminal = True
                    eb1.left.is_terminal = True

                    # Assign elbow bundle to edge
                    edge.elbow_bundle_v1 = eb1

                # Otherwise, retrieve elbow bundle around p1 from list
                else:
                    eb1 = elbow_bundles[i]

                    # Set orientation of eb1
                    eb1.orientation = bend_orientations[i - 1]

                # Construct elbow bundle around p2
                eb2 = ElbowBundle()
                elbow_bundles.append(eb2)
                p2.elbow_bundles.append(eb2)

                # Set eb2 properties
                eb2.point = p2
                eb2.size = 1
                eb2.thickness = edge.thickness

                # Set left and right for eb1 and eb2
                eb1.right = sb
                eb2.left = sb

                # If p2 = v2 is the last point, eb2 is a terminal elbow bundle, so we set right = left
                if i == len(edge.path) - 2:
                    eb2.right = sb

                    # Set is_terminal
                    eb2.is_terminal = True
                    eb2.right.is_terminal = True

                # Set sb properties
                sb.left = eb1
                sb.right = eb2
                sb.size = 1
                sb.thickness = edge.thickness

            self.straight_bundles.extend(straight_bundles)
            self.elbow_bundles.extend(elbow_bundles)

        # Construct terminal elbow bundles for obstacles
        for obstacle in self.instance.obstacles:
            # Construct elbow bundle
            eb = ElbowBundle()
            obstacle.elbow_bundles.append(eb)
            self.elbow_bundles.append(eb)

            # Set properties
            eb.point = obstacle
            eb.size = 1
            eb.thickness = 0
            eb.is_terminal = True

        # Sort elbow bundles around their points from closest to furthest
        for point in self.instance.graph.vertices + self.instance.obstacles:
            ebs = point.elbow_bundles

            if len(ebs) == 0:
                continue

            sorted_ebs = [ebs[0]]
            for new_eb in ebs[1:]:
                i = 0
                while i < len(sorted_ebs):
                    eb = sorted_ebs[i]
                    if new_eb.is_closer_than(eb):
                        sorted_ebs.insert(i, new_eb)
                        break

                    i += 1

                if i == len(sorted_ebs):
                    sorted_ebs.append(new_eb)

            # Set inner and layer_thickness based on the sorted order of the elbow bundles
            terminal_eb = sorted_ebs[0]
            terminal_eb.inner = None
            terminal_eb.layer_thickness = 0
            for i in range(1, len(sorted_ebs)):
                eb = sorted_ebs[i]
                eb.inner = sorted_ebs[i - 1]
                inner_thickness = eb.inner.thickness / 2 if eb.inner.is_terminal else eb.inner.thickness
                eb.layer_thickness = eb.inner.layer_thickness + inner_thickness

        # Revert all elbow bundles that form a left bend such that they become a right bend
        # This way, elbow bundles around the same point always have the same orientation
        for eb in self.elbow_bundles:
            if eb.orientation == 2:
                eb_left = eb.left
                eb.left = eb.right
                eb.right = eb_left
                eb.orientation = 1

        # Combine non-maximal straight bundles using union, which also combines the non-maximal elbow bundles
        i = 0
        while i < len(self.straight_bundles) - 1:
            sb1 = self.straight_bundles[i]

            # Do not bundle terminal straights/elbows
            if sb1.is_terminal:
                i += 1
                continue

            left_point = sb1.left.point
            right_point = sb1.right.point
            j = i + 1
            while j < len(self.straight_bundles):
                sb2 = self.straight_bundles[j]

                # Only union sb1 with non-terminal straight bundles sb2 that are associated with the same two points
                # Do not increment j after calling union since it deletes sb2
                if not sb2.is_terminal and sb2.is_associated_with(left_point) and sb2.is_associated_with(right_point):
                    self.union(sb1, sb2)
                else:
                    j += 1

            i += 1

    def split(self, sb, x, t):
        """
        Splits straight bundle sb(t) into a straight-elbow-straight bundle sequence sb(t), eb(t), sb2(t).
        Elbow bundle eb is degenerate and touches the elbow bundle x.

        :param sb: a StraightBundle object
        :param x: an ElbowBundle object
        :param t: the time between 0 and 1
        :returns: the new bundles sb, eb and sb2
        """
        sb2 = copy.copy(sb)
        self.straight_bundles.append(sb2)

        eb = ElbowBundle()
        self.elbow_bundles.append(eb)

        # Get the left and right endpoints of the backbone of sb
        p1, p2 = sb.get_backbone_endpoints(t)

        # Set left and right of the straight bundles based on orientation
        if orientation(x.point, p1, p2) == 1:
            sb.right = eb
            sb2.left = eb
        else:
            sb.left = eb
            sb2.right = eb

        # Due to the split, sb and sb2 may become or may no longer be terminal
        sb.is_terminal = sb.next(eb).is_terminal
        sb2.is_terminal = sb2.next(eb).is_terminal

        # Set properties of eb
        eb.point = x.point
        eb.left = sb
        eb.right = sb2
        eb.inner = x
        eb.size = sb.size
        eb.thickness = sb.thickness
        eb.layer_thickness = x.layer_thickness + x.thickness / 2 if x.is_terminal else x.layer_thickness + x.thickness

        # Update the elbow bundles on the right to point to sb2
        eb_right = sb2.next(eb)
        while eb_right is not None and eb_right.is_connected_to(sb):
            if eb_right.is_terminal:
                eb_right.left = sb2
                eb_right.right = sb2
            elif eb_right.left == sb:
                eb_right.left = sb2
            else:
                eb_right.right = sb2

            eb_right = eb_right.inner

        if not x.is_terminal:
            # Check if sb must unite with x's left straight bundle
            eb_left_of_x = x.left.next(x)
            if sb.is_associated_with(eb_left_of_x.point):
                self.union(sb, x.left)

            # Check if sb2 must unite with x's right straight bundle
            eb_right_of_x = x.right.next(x)
            if sb2.is_associated_with(eb_right_of_x.point):
                self.union(sb2, x.right)

        return sb, eb, sb2

    def merge(self, sb1, eb, sb2):
        """
        Merges straight-elbow-straight bundle sequence sb1, eb, sb2 into a single straight bundle.
        Elbow bundle eb must be degenerate and of zero length.
        Bundle sb1 will become the merged straight bundle.

        :param sb1: a StraightBundle object
        :param eb: an ElbowBundle object
        :param sb2: a StraightBundle object
        :returns: the new bundle sb1
        """
        if not sb1.is_connected_to(eb):
            raise Exception(f"Straight bundle {sb1} is not connected to elbow bundle {eb}")

        if not sb2.is_connected_to(eb):
            raise Exception(f"Straight bundle {sb2} is not connected to elbow bundle {eb}")

        # If sb1 or sb2 contains more segments than eb, we cannot directly merge the bundles together
        # Therefore, we first have to divide them such that all three bundles have the same size
        if sb1.size > eb.size:
            self.divide(sb1, eb)
        if sb2.size > eb.size:
            self.divide(sb2, eb)

        # Set right of sb1 to new right elbow bundle
        eb_right = sb2.next(eb)
        if sb1.right == eb:
            sb1.right = eb_right
        else:
            sb1.left = eb_right

        # Due to the merge, sb1 may become terminal
        sb1.is_terminal = sb1.is_terminal or sb2.is_terminal

        # Update the elbow bundles on the right to point to sb1
        while eb_right is not None and eb_right.is_connected_to(sb2):
            if eb_right.is_terminal:
                eb_right.left = sb1
                eb_right.right = sb1
            elif eb_right.left == sb2:
                eb_right.left = sb1
            else:
                eb_right.right = sb1

            eb_right = eb_right.inner

        # Delete eb and sb2
        self.elbow_bundles.remove(eb)
        self.straight_bundles.remove(sb2)

        return sb1

    def union(self, x, y):
        """
        Merges two non-maximal straight (resp. elbow) bundles x and y into a single straight (resp. elbow) bundle.
        The bundles must be associated with the same point(s) and share a line segment (resp. circular arc).
        Bundle x will become the united bundle.

        :param x: a StraightBundle (resp. ElbowBundle) object
        :param y: a StraightBundle (resp. ElbowBundle) object
        """
        if type(x) != type(y):
            raise Exception(f"Cannot union {type(x)} {x} and {type(y)} {y}")

        # Do not union terminal bundles
        if x.is_terminal or y.is_terminal:
            return

        x.size += y.size
        x.thickness += y.thickness

        if type(x) == type(y) == StraightBundle:
            # Set left of x
            if x.is_closer_than(y, x.left.point):
                x.left = y.left if x.has_same_orientation_as(y) else y.right

            # Set right of x
            if x.is_closer_than(y, x.right.point):
                x.right = y.right if x.has_same_orientation_as(y) else y.left

            # Update the left elbow bundles referencing y to reference x
            eb_left = y.left
            while eb_left is not None and eb_left.is_connected_to(y):
                if eb_left.right == y:
                    eb_left.right = x
                else:
                    eb_left.left = x

                eb_left = eb_left.inner

            # Update the right elbow bundles referencing y to reference x
            eb_right = y.right
            while eb_right is not None and eb_right.is_connected_to(y):
                if eb_right.left == y:
                    eb_right.left = x
                else:
                    eb_right.right = x

                eb_right = eb_right.inner

            # Check to see if two elbows on the left must merge
            eb_left = x.left
            eb_left_inner = eb_left.inner
            if not eb_left.is_terminal:
                while eb_left.is_connected_to(x) and eb_left_inner.is_connected_to(x) and not eb_left_inner.is_terminal:
                    if eb_left.next(x) == eb_left_inner.next(x):
                        self.union(eb_left, eb_left_inner)
                    else:
                        eb_left = eb_left_inner

                    eb_left_inner = eb_left.inner

            # Check to see if two elbows on the right must merge
            eb_right = x.right
            eb_right_inner = eb_right.inner
            if not eb_right.is_terminal:
                while (eb_right.is_connected_to(x) and eb_right_inner.is_connected_to(x)
                       and not eb_right_inner.is_terminal):
                    if eb_right.next(x) == eb_right_inner.next(x):
                        self.union(eb_right, eb_right_inner)
                    else:
                        eb_right = eb_right_inner

                    eb_right_inner = eb_right.inner

            # Remove the degenerate straight bundle y
            self.straight_bundles.remove(y)
        else:
            # If y is closer to the point than x, we only need to update inner and layer_thickness of x
            if y.is_closer_than(x):
                x.inner = y.inner
                x.layer_thickness = y.layer_thickness

            # Otherwise, we need to update the straight bundles referencing y to reference x
            else:
                sb_left = y.left
                if sb_left.right == y:
                    sb_left.right = x
                else:
                    sb_left.left = x

                sb_right = y.right
                if sb_right.left == y:
                    sb_right.left = x
                else:
                    sb_right.right = x

            # Remove the degenerate elbow bundle y
            self.elbow_bundles.remove(y)

    def divide(self, sb, eb):
        """
        Splits straight bundle sb at the adjacent elbow bundle eb into two interior-disjoint straight bundles.
        The new straight bundles have the same associated point and (temporarily) share a straight segment.
        This is used before removing the degenerate bundle eb when merging sb with eb and another straight bundle.
        The two divided straight bundles will no longer be adjacent to the same points after completing the merge.

        :param sb: a StraightBundle object
        :param eb: an ElbowBundle object
        """
        # sb will become the outermost straight bundle adjacent to eb with the same size as eb
        # Therefore, we create a new straight bundle sb2 which will form the remaining bundle after tearing off sb
        sb2 = copy.copy(sb)
        self.straight_bundles.append(sb2)

        # Connect sb2 to the inner elbow bundle of eb
        if sb2.right == eb:
            sb2.right = eb.inner
        else:
            sb2.left = eb.inner

        # Update the inner elbow bundles referencing sb to reference sb2
        eb_inner = eb.inner
        while eb_inner is not None and eb_inner.is_connected_to(sb):
            if eb_inner.left == sb:
                eb_inner.left = sb2
            else:
                eb_inner.right = sb2

            eb_inner = eb_inner.inner

        sb.size = eb.size
        sb.thickness = eb.thickness

        sb2.size -= sb.size
        sb2.thickness -= sb.thickness

        # Consider the next (outermost) elbow bundle on the other side of the straight bundles
        eb_next = sb.next(eb)
        size = eb_next.size
        thickness = eb_next.thickness

        # Determine the directions of the elbow bundles
        dir_eb = eb.left == sb
        dir_eb_next = eb_next.right == sb

        # We already assumed that sb will be connected to eb
        # If the elbow bundles on either side have the same orientation, eb_next will be the other elbow bundle of sb
        if dir_eb == dir_eb_next:
            # Go over the elbow bundles from eb_next inwards until their combined size is at least the size of sb
            while size < sb.size:
                eb_next = eb_next.inner

                size += eb_next.size
                thickness += eb_next.thickness

            # If size is larger than sb's size, the current elbow bundle eb_next must be split
            if size > sb.size:
                # eb_next will become the elbow bundle that is still adjacent to sb
                # Therefore, we create a new elbow bundle eb_new which will form the remaining elbow bundle
                eb_new = copy.copy(eb_next)
                self.elbow_bundles.append(eb_new)

                eb_next.inner = eb_new

                eb_new.size = size - sb.size
                eb_new.thickness = thickness - sb.thickness

                eb_next.size -= eb_new.size
                eb_next.thickness -= eb_new.thickness
                eb_next.layer_thickness = eb_new.layer_thickness + eb_new.thickness

            # eb_next is the last elbow bundle adjacent to sb
            # Therefore, its inner elbow bundle is the outermost elbow bundle of sb2
            eb_next = eb_next.inner
            if sb2.right == eb.inner:
                sb2.left = eb_next
            else:
                sb2.right = eb_next

            # Update the inner elbow bundles referencing sb to reference sb2
            while eb_next is not None and eb_next.is_connected_to(sb):
                if eb_next.right == sb:
                    eb_next.right = sb2
                else:
                    eb_next.left = sb2

                eb_next = eb_next.inner

        # Otherwise, eb_next will be the other elbow bundle of sb2
        else:
            # Go over the elbow bundles from eb_next inwards until their combined size is at least the size of sb2
            while size < sb2.size:
                # Update the inner elbow bundles referencing sb to reference sb2
                if eb_next.left == sb:
                    eb_next.left = sb2
                else:
                    eb_next.right = sb2

                eb_next = eb_next.inner

                size += eb_next.size
                thickness += eb_next.thickness

            # If size is larger than sb2's size, the current elbow bundle eb_next must be split
            if size > sb2.size:
                # eb_next will become the elbow bundle that is still adjacent to sb2
                # Therefore, we create a new elbow bundle eb_new which will form the remaining elbow bundle
                eb_new = copy.copy(eb_next)
                self.elbow_bundles.append(eb_new)

                eb_next.inner = eb_new

                eb_new.size = size - sb2.size
                eb_new.thickness = thickness - sb2.thickness

                eb_next.size -= eb_new.size
                eb_next.thickness -= eb_new.thickness
                eb_next.layer_thickness = eb_new.layer_thickness + eb_new.thickness

            # eb_next is the last elbow bundle adjacent to sb2, but it does not reference sb2 yet
            if eb_next.left == sb:
                eb_next.left = sb2
            else:
                eb_next.right = sb2

            # The inner elbow bundle of eb_next is the outermost elbow bundle of sb
            eb_next = eb_next.inner
            if sb.left == eb:
                sb.right = eb_next
            else:
                sb.left = eb_next

    def tear(self, b):
        """
        Removes the top segment from the given straight bundle.

        :param b: a StraightBundle object
        """
        if b.size == 1:
            return

        # Construct a new straight bundle for the segment to be torn off b
        c = StraightBundle()
        self.straight_bundles.append(c)

        # Set the sizes of c and b
        c.size = 1
        b.size -= 1

        # Get the elbow bundles left and right from b
        eb_left = b.left
        eb_right = b.right

        # Determine the directions of the elbow bundles
        dir_eb_left = eb_left.right == b
        dir_eb_right = eb_right.left == b

        # Let eb_left become the outermost left elbow and attach to c
        c.left = eb_left
        if eb_left.right == b:
            eb_left.right = c
        else:
            eb_left.left = c

        # If eb_left only contains one elbow, the inner of eb_left becomes b's outermost left elbow
        if eb_left.size == 1:
            b.left = eb_left.inner

        # Otherwise, we need to tear off one elbow from eb_left
        else:
            # Construct a new elbow bundle that will be inner of eb_left
            eb_inner = copy.copy(eb_left)
            self.elbow_bundles.append(eb_inner)

            # Set the sizes of eb_left and eb_inner
            eb_left.size = 1
            eb_inner.size -= 1

            # Connect eb_inner to eb_left and b
            eb_left.inner = eb_inner
            b.left = eb_inner
            if eb_inner.right == c:
                eb_inner.right = b
            else:
                eb_inner.left = b

        # We already tore off the left outermost elbow and attached it to c
        # If the elbow bundle on the right has the same orientation as the left one, c's right elbow is also outermost
        if dir_eb_left == dir_eb_right:
            # Let eb_right become the outermost right elbow and attach to c
            c.right = eb_right
            if eb_right.left == b:
                eb_right.left = c
            else:
                eb_right.right = c

            # If eb_right only contains one elbow, the inner of eb_right becomes b's outermost right elbow
            if eb_right.size == 1:
                b.right = eb_right.inner

            # Otherwise, we need to tear off one elbow from eb_right
            else:
                # Construct a new elbow bundle that will be inner of eb_right
                eb_inner = copy.copy(eb_right)
                self.elbow_bundles.append(eb_inner)

                # Set the sizes of eb_right and eb_inner
                eb_right.size = 1
                eb_inner.size -= 1

                # Connect eb_inner to eb_right and b
                eb_right.inner = eb_inner
                b.right = eb_inner
                if eb_inner.left == c:
                    eb_inner.left = b
                else:
                    eb_inner.right = b

        # Otherwise, c's right elbow is innermost, so we need to tear off the right innermost elbow adjacent to b
        else:
            # Find the right innermost elbow bundle adjacent to b
            while eb_right.inner.is_connected_to(b):
                eb_right = eb_right.inner

            # Let eb_inner become the innermost right elbow
            # If eb_right only contains one elbow, eb_inner simply becomes eb_right
            if eb_right.size == 1:
                eb_inner = eb_right

            # Otherwise, we need to tear off one elbow from eb_right
            else:
                # Construct a new elbow bundle that will be inner of eb_right
                eb_inner = copy.copy(eb_right)
                self.elbow_bundles.append(eb_inner)

                # Set the sizes of eb_inner and eb_right
                eb_inner.size = 1
                eb_right.size -= 1

                # Connect eb_inner to eb_right
                # We do not connect it to b as eb_right remains the right outermost elbow bundle of b
                eb_right.inner = eb_inner

            # Attach eb_inner to c
            c.right = eb_inner
            if eb_inner.left == b:
                eb_inner.left = c
            else:
                eb_inner.right = c

    def unzip(self):
        """
        Unzips the bundles such that each bundle consists of exactly one segment.
        """
        # We unzip the bundles by repeatedly tearing segments from the straight bundles until they all have one segment
        # By definition, if all straight bundles have exactly one segment, all elbow bundles must also have one segment
        straight_bundles = self.straight_bundles.copy()
        for sb in straight_bundles:
            # While sb contains more than one straight, tear the top segment off
            # Since tear creates exactly one new singular straight bundle, we do not need to unzip the torn bundles
            while sb.size > 1:
                self.tear(sb)

        # The process of tearing destroys the thickness information
        # Therefore, we need to reset the thickness of the non-terminal bundles
        for edge in self.instance.graph.edges:
            # Start from the first non-terminal bundle
            prev_bundle = edge.elbow_bundle_v1.right
            current_bundle = prev_bundle.right
            while not current_bundle.is_terminal:
                current_bundle.thickness = edge.thickness

                current_b = current_bundle
                current_bundle = current_bundle.next(prev_bundle)
                prev_bundle = current_b

        # Now we can also recompute the layer_thickness of the elbow bundles using the inner and thickness fields
        for eb in self.elbow_bundles:
            layer_thickness = 0
            eb_inner = eb.inner
            while eb_inner is not None:
                layer_thickness += eb_inner.thickness / 2 if eb_inner.is_terminal else eb_inner.thickness
                eb_inner = eb_inner.inner

            eb.layer_thickness = layer_thickness

    def __contains__(self, bundle):
        if type(bundle) == StraightBundle:
            return bundle in self.straight_bundles
        else:
            return bundle in self.elbow_bundles

    def __str__(self):
        result = "Straight bundles:\n"
        for sb in self.straight_bundles:
            result += f"- {sb}"
            result += " [terminal]\n" if sb.is_terminal else "\n"

        result += "\nElbow bundles:\n"
        for eb in self.elbow_bundles:
            result += f"- {eb}"
            result += " [terminal]\n" if eb.is_terminal else "\n"
        result = result[:-1]

        return result
