from delaunay_triangulation import DelaunayTriangulation


def compute_funnel(sequence, edge):
    """
    Computes the funnel of the reduced crossing sequence through the Delaunay triangulation.

    :param sequence: a list describing the reduced crossing sequence of the edge
    :param edge: an Edge object
    :returns a list describing the funnel of the edge
    """
    return []


def compute_shortest_path(funnel, edge):
    """
    Computes the shortest path through the funnel.

    :param funnel: a list describing the funnel of the edge through the Delaunay triangulation
    :param edge: an Edge object
    :returns: a list describing the shortest homotopic path of the edge
    """
    return edge.path


class Homotopy:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

        # Compute the Delaunay triangulation on the vertices and obstacles
        self.dt = DelaunayTriangulation(self.instance)

    def compute_initial_triangle(self, edge):
        """
        Computes the first triangle of the Delaunay triangulation that the edge moves through.
        Edge links located on the boundary of a triangle are not considered to be moving 'through' the triangle.

        :param edge: an Edge object
        :returns: the first triangle and the index of the first edge link moving through the triangle
        """
        # Retrieve Delaunay vertex corresponding to first edge vertex
        dt_v1 = self.dt.get_delaunay_vertex_from_point(edge.v1)

        i = 0

        # Find the first edge link that moves through a triangle adjacent to v1
        while i < len(edge.path) - 1:
            target = edge.path[i + 1]

            # For each triangle adjacent to v1, check if it contains the edge link
            for dt_edge in dt_v1.outgoing_edges:
                if dt_edge.orientation(target) == dt_edge.prev.orientation(target) == 2:
                    return dt_edge.face, i

            i += 1

        return None, i

    def compute_crossing_sequence(self, edge):
        """
        Computes the (unreduced) sequence of half-edges of the Delaunay triangulation crossed by the edge.
        Only records the half-edges that are crossed when exiting their corresponding triangle.

        :param edge: an Edge object
        :returns: a list describing the crossing sequence of the edge
        """
        sequence = []

        # Compute initial triangle and index of first edge link moving 'through' the triangle
        current_triangle, i = self.compute_initial_triangle(edge)

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
                current_triangle = crossed_dt_edge.twin.face
                crossed_dt_edge = current_triangle.exited_by(p1, p2)

            i += 1

        return sequence

    def reduce_crossing_sequence(self, sequence, edge):
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

        # Retrieve Delaunay vertices corresponding to edge vertices
        dt_v1 = self.dt.get_delaunay_vertex_from_point(edge.v1)
        dt_v2 = self.dt.get_delaunay_vertex_from_point(edge.v2)

        # Remove redundant edge links incident to vertices
        while reduced_sequence:
            if any(e in dt_v1.outgoing_edges for e in [reduced_sequence[0], reduced_sequence[0].twin]):
                reduced_sequence.pop(0)
            elif any(e in dt_v2.outgoing_edges for e in [reduced_sequence[-1], reduced_sequence[-1].twin]):
                reduced_sequence.pop()
            else:
                break

        return reduced_sequence

    def compute_shortest_edges(self):
        """
        Computes the shortest homotopic edges.
        """
        for edge in self.instance.graph.edges:
            print(f"\nEdge: {edge}")
            # Compute the crossing sequence of the edge
            sequence = self.compute_crossing_sequence(edge)

            # Reduce the crossing sequence
            reduced_sequence = self.reduce_crossing_sequence(sequence, edge)

            print("Reduced crossing sequence:")
            for e in reduced_sequence:
                print(e)

            # Compute the funnel of the reduced crossing sequence
            funnel = compute_funnel(reduced_sequence, edge)

            # Compute the shortest path through the funnel
            shortest_path = compute_shortest_path(funnel, edge)

            # Assign the shortest homotopic path to the edge
            edge.path = shortest_path
