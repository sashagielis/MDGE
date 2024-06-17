import copy
import math

from utils import angle, on_segment, orientation


class StraightBundle:
    """
    A set of straights associated with the same two points.
    """
    def __init__(self):
        # Attributes from paper
        self.left = None  # The outermost adjacent elbow bundle on the left
        self.right = None  # The outermost adjacent elbow bundle on the right
        self.size = None  # The number of segments contained in the bundle
        self.thickness = None  # The total thickness of the segments in the bundle

        # Additional attributes
        self.is_terminal = False  # Whether it is a terminal straight bundle (i.e. connected to a terminal elbow bundle)
        self.angle = None  # The angle of the straight bundle in radians

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

    def __str__(self):
        return f"{self.left} -> {self.right}"


class ElbowBundle:
    """
    A set of elbows associated with the same point and sharing the same straight bundle on both ends.
    A terminal elbow forms its own bundle and is not merged with other bundles.
    """
    def __init__(self):
        # Attributes from paper
        self.point = None  # The associated point
        self.left = None  # The adjacent straight bundle on the left (left = right for terminal elbows)
        self.right = None  # The adjacent straight bundle on the right (left = right for terminal elbows)
        self.inner = None  # The elbow bundle just inside it (None for terminal elbows)
        self.size = None  # The number of segments contained in the bundle (1 for terminal elbows)
        self.thickness = None  # The total thickness of the segments in the bundle (0 for obstacle elbows)
        self.layer_thickness = None  # The total thickness of the paths between eb and eb.point (0 for terminal elbows)

        # Additional attributes
        self.is_terminal = False  # Whether it is a terminal elbow bundle (i.e., associated with a vertex or obstacle)
        self.left_angle = None  # The left angle of the annular wedge in radians (None for terminal elbows)
        self.right_angle = None  # The right angle of the annular wedge in radians (None for terminal elbows)
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
            crossing_sequence = edge.crossing_sequence
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

                # Set angle of sb
                sb.angle = angle(p1, p2)

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

                    # Set is_terminal and orientation
                    eb1.is_terminal = True
                    eb1.left.is_terminal = True

                    # Assign elbow bundle to edge
                    edge.elbow_bundle_v1 = eb1

                # Otherwise, retrieve elbow bundle around p1 from list
                else:
                    eb1 = elbow_bundles[i]

                    # Set orientation of eb1
                    eb1.orientation = bend_orientations[i - 1]

                    # Set left and right angle of eb1 based on orientation
                    prev_sb = eb1.left
                    rotation = 0.5 * math.pi
                    if eb1.orientation == 1:
                        eb1.left_angle = prev_sb.angle + rotation
                        eb1.right_angle = sb.angle + rotation
                    else:
                        eb1.left_angle = sb.angle - rotation
                        eb1.right_angle = prev_sb.angle - rotation

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

                    # Set is_terminal and orientation
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

    def split(self, sb, x):
        """
        Splits straight bundle sb into a straight-elbow-straight bundle sequence sb1, eb, sb2.
        Elbow bundle eb is degenerate and touches the elbow bundle x.

        :param sb: a StraightBundle object
        :param x: an ElbowBundle object
        """
        sb2 = sb.copy()
        self.straight_bundles.append(sb2)

        eb = ElbowBundle()
        self.elbow_bundles.append(eb)

        sb.right = eb
        sb2.left = eb

        eb.left = sb
        eb.right = sb2
        eb.inner = x

        eb.size = sb.size
        eb.thickness = sb.thickness

        eb.layer_W = x.layer_W + x.thickness

        eb_right = sb2.right

        while eb_right.left == sb:
            eb_right.left = sb2
            eb_right = eb_right.inner

        if x.left.left.point is sb.left.point:
            self.union(sb, x.left)

        if x.right.right.point is sb2.right.point:
            self.union(sb2, x.right)

    def merge(self, sb1, eb, sb2):
        """
        Merges straight-elbow-straight bundle sequence sb1, eb, sb2 into a single straight bundle sb.
        Elbow bundle eb must be degenerate and of zero length.

        :param sb1: a StraightBundle object
        :param eb: an ElbowBundle object
        :param sb2: a StraightBundle object
        """
        return

    def union(self, x, y):
        """
        Merges two non-maximal straight (resp. elbow) bundles x and y into a single straight (resp. elbow) bundle.
        The bundles must be associated with the same point(s) and share a line segment (resp. circular arc).
        Bundle x will become the united bundle.

        :param x: a StraightBundle (resp. ElbowBundle) object
        :param y: a StraightBundle (resp. ElbowBundle) object
        """
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
            while eb_left.is_connected_to(y) and not eb_left.is_terminal:
                if eb_left.right == y:
                    eb_left.right = x
                else:
                    eb_left.left = x

                eb_left = eb_left.inner

            # Update the right elbow bundles referencing y to reference x
            eb_right = y.right
            while eb_right.is_connected_to(y) and not eb_right.is_terminal:
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
                while eb_right.is_connected_to(x) and eb_right_inner.is_connected_to(
                        x) and not eb_right_inner.is_terminal:
                    if eb_right.next(x) == eb_right_inner.next(x):
                        self.union(eb_right, eb_right_inner)
                    else:
                        eb_right = eb_right_inner

                    eb_right_inner = eb_right.inner

            # Remove the degenerate straight bundle y
            self.straight_bundles.remove(y)
        elif type(x) == type(y) == ElbowBundle:
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
        else:
            raise Exception(f"Cannot union {type(x)} {x} and {type(y)} {y}")

    def divide(self, sb, eb):
        """
        Splits straight bundle sb at the adjacent elbow bundle eb into two interior-disjoint straight bundles sb1 and sb2.
        Straight bundles sb1 and sb2 have the same associated vertices and (temporarily) share a straight segment.
        This is used in preparation for removing the degenerate bundle eb by merging sb1 with the other straight bundle adjacent to eb.
        The two divided straight bundles will then no longer be adjacent to the same vertices.

        :param sb: a StraightBundle object
        :param eb: an ElbowBundle object
        """
        return

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

                # If necessary, switch left and right such that we always move along the bundles towards the right
                if current_bundle.right == prev_bundle:
                    current_bundle.right = current_bundle.left
                    current_bundle.left = prev_bundle

                    # Switch the elbow orientation from clockwise to counterclockwise
                    if type(current_bundle) == ElbowBundle:
                        current_bundle.orientation = 2

                prev_bundle = current_bundle
                current_bundle = current_bundle.right

        # Now we can also recompute the layer_thickness of the elbow bundles using the inner and thickness fields
        for eb in self.elbow_bundles:
            layer_thickness = 0
            eb_inner = eb.inner
            while eb_inner is not None:
                layer_thickness += eb_inner.thickness / 2 if eb_inner.is_terminal else eb_inner.thickness
                eb_inner = eb_inner.inner

            eb.layer_thickness = layer_thickness

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
