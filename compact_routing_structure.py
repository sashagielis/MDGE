import math

from utils import orientation, angle


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
        self.angle = None  # The angle of the straight bundle in radians

    def __str__(self):
        return f"{self.left} -> {self.right}"


class ElbowBundle:
    """
    A set of elbows associated with the same point and sharing the same straight bundles on both ends.
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
        self.is_terminal = False  # Whether it is a terminal elbow bundle
        self.left_angle = None  # The left angle of the annular wedge in radians (None for terminal elbows)
        self.right_angle = None  # The right angle of the annular wedge in radians (None for terminal elbows)
        self.orientation = 0  # The orientation of the bend, where 0 : terminal, 1 : clockwise, 2 : counterclockwise

    def is_closer_than(self, eb):
        """
        Determines whether the elbow bundle is closer to its associated point than the given elbow bundle eb.
        The elbow bundles must be associated with the same point.
        Assumes that if self.inner == eb.inner == None, the inner field is not yet initialized.
        Cannot handle collinear points well...

        :param eb: an ElbowBundle object
        """
        if self.point != eb.point:
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

        # Check if self is inside eb
        inner_eb = eb.inner
        while inner_eb is not None:
            if inner_eb == self:
                return True

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
        while current_self.point == current_eb.point:
            current_self = self.right.right if not revert_self else self.left.left
            current_eb = eb.right.right if not revert_eb else eb.left.left

        # Determine the elbow bundles just before the current ones
        prev_self = self.left.left if not revert_self else self.right.right
        prev_eb = eb.left.left if not revert_eb else eb.right.right

        # If current_eb ended at a terminal elbow, self is closest if its last bend was a left turn
        if prev_self.point == current_eb.point:
            return prev_self.orientation == 2

        # If current_self ended at a terminal elbow, self is closest if eb's last bend was a right turn
        elif prev_eb.point == current_self.point:
            return prev_eb.orientation == 1

        # Otherwise, self is closest if its current point is right of eb's last segment
        else:
            return orientation(prev_eb.point, current_eb.point, current_self.point) == 1

    def __str__(self):
        return str(self.point)


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
                    eb1.is_terminal = True

                    # Assign elbow bundle to edge
                    edge.elbow_bundle_v1 = eb1

                # Otherwise, retrieve elbow bundle around p1 from list
                else:
                    eb1 = elbow_bundles[i]

                    # Iterate over crossings in crossing sequence until first crossing with p1 as endpoint is found
                    # The crossings around an elbow all have the elbow point on the same side of the half-edge
                    # Therefore, the elbow orientation results from whether the point is origin/target of the crossing
                    found_orientation = False
                    while not found_orientation:
                        crossing = crossing_sequence[crossing_index]

                        # If p1 is the origin of the crossing, eb1 is a right turn
                        if crossing.origin == p1:
                            eb1.orientation = 1
                            found_orientation = True

                        # If p1 is the target of the crossing, eb1 is a left turn
                        elif crossing.target == p1:
                            eb1.orientation = 2
                            found_orientation = True

                        # Otherwise, check next crossing
                        else:
                            crossing_index += 1

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
                    eb2.is_terminal = True

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

            # Set inner and layer_thickness
            terminal_eb = sorted_ebs[0]
            terminal_eb.inner = None
            terminal_eb.layer_thickness = 0
            for i in range(1, len(sorted_ebs)):
                eb = sorted_ebs[i]
                eb.inner = sorted_ebs[i - 1]
                inner_thickness = eb.inner.thickness / 2 if eb.inner.is_terminal else eb.inner.thickness
                eb.layer_thickness = eb.inner.layer_thickness + inner_thickness

        # TODO: Union non-maximal bundles

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

        if x.left.left.point == sb.left.point:
            self.union(sb, x.left)

        if x.right.right.point == sb2.right.point:
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
        Merges two non-maximal straight (resp. elbow) bundles x and y into a single straight (resp. elbow) bundle z.
        The bundles must be associated with the same vertices and share a line segment (resp. circular arc).

        :param x: a StraightBundle (resp. ElbowBundle) object
        :param y: a StraightBundle (resp. ElbowBundle) object
        """
        return

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

    def __str__(self):
        result = "Straight bundles:\n"
        for sb in self.straight_bundles:
            result += f"- {sb}\n"

        result += "\nElbow bundles:\n"
        for eb in self.elbow_bundles:
            result += f"- {eb}"
            result += " [terminal]\n" if eb.is_terminal else "\n"
        result = result[:-1]

        return result
