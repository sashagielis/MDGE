class Obstacle:
    def __init__(self, path, color):
        self.path = path
        self.color = color

    def __str__(self):
        result = "["
        for node in self.path:
            result += f"({node[0]}, {node[1]}), "
        result = result[:-2] + "]"

        return result
