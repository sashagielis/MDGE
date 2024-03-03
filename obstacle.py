class Obstacle:
    def __init__(self, path, fill_color, stroke_color='black'):
        self.path = path
        self.fill_color = fill_color
        self.stroke_color = stroke_color

    def __str__(self):
        result = "["
        for point in self.path:
            result += str(point) + ", "
        result = result[:-2] + "]"

        return result
