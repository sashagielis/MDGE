class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.outgoing_dt_edges = []  # Set of half-edges leaving the point in a Delaunay triangulation

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, value):
        return Point(self.x * value, self.y * value)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, value):
        return Point(self.x / value, self.y / value)

    def __str__(self):
        return f"({float(self.x)}, {float(self.y)})"
