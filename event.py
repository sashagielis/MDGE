class Event:
    """
    An event of the growing algorithm at time t.
    """
    def __init__(self, t, bundles):
        """
        :param t: the time between 0 and 1
        :param bundles: a list of StraightBundle and ElbowBundle objects involved in the event
        """
        self.t = t
        self.bundles = bundles

    def __lt__(self, other):
        return self.t < other.t or (self.t == other.t and type(self) == MergeEvent)


class SplitEvent(Event):
    """
    A split event stating that an elbow bundle splits a straight bundle at time t.
    """
    def __init__(self, t, sb, eb):
        """
        :param t: the time between 0 and 1
        :param sb: a StraightBundle object
        :param eb: an ElbowBundle object
        """
        super().__init__(t, [sb, eb])
        self.sb = sb
        self.eb = eb

    def is_valid(self):
        """
        Determines whether the elbow bundle eb should still split the straight bundle sb at time t.
        """
        return self.eb.splits(self.sb, self.t)

    def __str__(self):
        return f"t = {self.t}: elbow bundle {self.eb} splits straight bundle {self.sb}"


class MergeEvent(Event):
    """
    A merge event stating that an elbow bundle is merged with its adjacent straight bundles at time t.
    """
    def __init__(self, t, eb):
        """
        :param t: the time between 0 and 1
        :param eb: an ElbowBundle object
        """
        super().__init__(t, [eb])
        self.eb = eb

    def is_valid(self):
        """
        Determines whether the elbow bundle eb should still be merged with its adjacent straight bundles at time t.
        """
        return self.eb.merges(self.t)

    def __str__(self):
        return f"t = {self.t}: elbow bundle {self.eb} merges with straight bundles {self.eb.left} and {self.eb.right}"
