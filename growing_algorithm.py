from queue import PriorityQueue

from compact_routing_structure import CompactRoutingStructure, ElbowBundle, StraightBundle
from event import MergeEvent, SplitEvent


class GrowingAlgorithm:
    def __init__(self, instance, dt):
        """
        :param instance: a SimplifiedInstance object
        :param dt: the time interval of the growing process
        """
        self.instance = instance
        self.dt = dt
        self.crs = CompactRoutingStructure(instance)
        self.queue = PriorityQueue()

    def update_queue(self, b, t):
        """
        Computes the next event(s) of the given bundle after the given time t and inserts them into the queue.

        :param b: a StraightBundle or ElbowBundle object
        :param t: the time between 0 and 1
        """
        def binary_search(start_time, no_iter, func, *args):
            """
            Performs a binary search to approximate the actual event time.
            The value of func(args, start_time) must be True, while func(args, start_time - dt) must be False.

            :param start_time: the time between 0 and 1
            :param no_iter: the number of iterations
            :param func: a function call to an event condition
            :param args: the arguments of func, except for the time parameter
            :returns: the approximated event time
            """
            # We start with the interval [t1, t2] = [start_time - dt, start_time]
            # In each iteration, check if the event is still valid at time t' = (t2 + t1) / 2
            # If it is, decrease the interval to [t1, t'], otherwise to [t', t2]
            new_t = start_time
            for i in range(no_iter):
                step = self.dt / (2 ** (i + 1))
                new_t -= step

                # If the event is not valid at the new time, go back to the previous time
                if not func(*args, new_t):
                    new_t += step

            return new_t

        def compute_next_split_event():
            """
            Computes the next split event of the given bundle after time t.
            """
            # Determine which bundles to consider for the split
            if type(b) == StraightBundle:
                split_bundles = self.crs.elbow_bundles
            else:
                split_bundles = self.crs.straight_bundles

            # Grow time by dt until the next split event of b is found
            # Since we roughly approximate the time, there may be multiple bundles that split with b at the found time
            # Therefore, we first find all such bundles
            next_split_bundles = []
            next_split_time = 1
            for bundle in split_bundles:
                if b.is_connected_to(bundle):
                    continue

                current_t = t + self.dt
                while current_t <= next_split_time:
                    if b.splits(bundle, current_t):
                        if current_t < next_split_time:
                            next_split_time = current_t
                            next_split_bundles = []

                        next_split_bundles.append(bundle)
                        break
                    else:
                        current_t += self.dt

            if not next_split_bundles:
                return None

            # Use binary search to determine the actual split times of the found bundles
            # The bundle with the smallest actual split time then forms the next split event with b
            next_split_bundle = None
            first_t = next_split_time
            for bundle in next_split_bundles:
                # Use binary search to approximate the exact split time
                new_t = binary_search(next_split_time, 100, b.splits, bundle)

                if new_t <= first_t:
                    first_t = new_t
                    next_split_bundle = bundle

            next_split_time = first_t
            if type(b) == StraightBundle:
                sb = b
                eb = next_split_bundle
            else:
                sb = next_split_bundle
                eb = b

            return SplitEvent(next_split_time, sb, eb)

        def compute_next_merge_event():
            """
            Computes the next merge event of the given elbow bundle after time t.
            Bundle b must be an elbow bundle.
            """
            # Grow time by dt until the next merge event of b is found
            current_t = t + self.dt
            while current_t <= 1:
                if b.merges(current_t):
                    # Use binary search to approximate the exact merge time
                    next_merge_time = binary_search(current_t, 100, b.merges)

                    return MergeEvent(next_merge_time, b)

                current_t += self.dt

            return None

        # Compute the next split event of b and insert it into the queue
        split_event = compute_next_split_event()
        if split_event is not None:
            self.queue.put(split_event)

        if type(b) == ElbowBundle and not b.is_terminal:
            # Compute the next merge event of b and insert it into the queue
            merge_event = compute_next_merge_event()
            if merge_event is not None:
                self.queue.put(merge_event)

    def compute_thick_edges(self, print_events=False):
        """
        Computes the thick homotopic edges.
        Follows growing algorithm by Duncan et al. (https://doi.org/10.1142/S0129054106004315).

        :param print_events: whether the executed events should be printed
        """
        # Initialize the event queue
        for eb in self.crs.elbow_bundles:
            self.update_queue(eb, 0)

        if print_events:
            if self.queue.empty():
                print("No growing events")
            else:
                print("Growing events:")

        while not self.queue.empty():
            event = self.queue.get()

            # Handle invalid events
            if any(b not in self.crs for b in event.bundles) or not event.is_valid():
                # Compute the next events for the bundles that still exist in the crs
                for b in event.bundles:
                    if b in self.crs:
                        self.update_queue(b, event.t)

                # Continue with the next event
                continue

            if print_events:
                print(event)

            # Execute the event
            if type(event) == SplitEvent:
                sb1, eb, sb2 = self.crs.split(event.sb, event.eb, event.t)

                self.update_queue(event.eb, event.t)
                self.update_queue(sb1, event.t)
                self.update_queue(eb, event.t)
                self.update_queue(sb2, event.t)
            else:
                sb = self.crs.merge(event.eb.left, event.eb, event.eb.right)

                self.update_queue(sb, event.t)

        # Unzip the bundles to obtain the singular thick edges
        self.crs.unzip()
