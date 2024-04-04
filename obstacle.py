import itertools

from point import Point


class Obstacle:
    id_iter = itertools.count()

    def __init__(self, fill_color='grey', stroke_color='black'):
        self.id = next(self.id_iter)
        self.fill_color = fill_color
        self.stroke_color = stroke_color


class PointObstacle(Point, Obstacle):
    def __init__(self, point, fill_color='grey', stroke_color='black'):
        Point.__init__(self, point.x, point.y)
        Obstacle.__init__(self, fill_color, stroke_color)

        self.displacement = 0


class PolygonalObstacle(Obstacle):
    def __init__(self, path, fill_color='grey', stroke_color='black'):
        super().__init__(fill_color, stroke_color)

        self.path = path

    def __str__(self):
        result = "["
        for point in self.path:
            result += str(point) + ", "
        result = result[:-2] + "]"

        return result
