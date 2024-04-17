from itertools import pairwise

from delaunay_triangulation import DelaunayTriangulation


def reduce_crossing_sequence(sequence):
    """
    Reduces a crossing sequence by removing adjacent pairs of equal crossings.

    :param sequence: a list of sequentially crossed Delaunay edges
    :returns the reduced crossing sequence
    """
    reduced_sequence = []

    for dt_edge in sequence:
        if reduced_sequence and reduced_sequence[-1] == dt_edge:
            reduced_sequence.pop()
        else:
            reduced_sequence.append(dt_edge)

    return reduced_sequence


class Homotopy:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

    def compute_shortest_edges(self):
        """
        Computes the homotopic shortest edges.
        """
        dt = DelaunayTriangulation(self.instance)
        print(dt)

        # TODO: Compute crossing sequence of each edge
        reduced_crossing_sequences = []
        for edge in self.instance.graph.edges:
            sequence = []
            current_face = None

            for p, q in pairwise(edge.path):
                crossed_dt_edge = current_face.crossed_by(p, q)
                while crossed_dt_edge is not None:
                    sequence.append(crossed_dt_edge)
                    current_face = crossed_dt_edge.twin.face
                    crossed_dt_edge = current_face.crossed_by(p, q)

            # Reduce crossing sequence
            reduced_sequence = reduce_crossing_sequence(sequence)
            reduced_crossing_sequences.append(reduced_sequence)

        # TODO: grow funnels

        # TODO: compute shortest paths from funnels
