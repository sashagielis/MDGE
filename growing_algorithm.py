from compact_routing_structure import CompactRoutingStructure


class GrowingAlgorithm:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance

    def compute_thick_edges(self):
        """
        Computes the thick homotopic edges.
        Follows growing algorithm by Duncan et al. (https://doi.org/10.1142/S0129054106004315).
        """
        crs = CompactRoutingStructure(self.instance)

        # TODO: Grow edges

        # Unzip the bundles to obtain the singular thick edges
        crs.unzip()

        print(crs)
