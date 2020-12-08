class TrafficElement:
    """
    Data class for detected elements
    """

    def __init__(self, x: int = 0, y: int = 0):
        """

        :param x: x coord in image
        :param y: y coord in image
        """
        self.x = x
        self.y = y


class SpeedSignElement(TrafficElement):
    def __init__(self, x: int = 0, y: int = 0, limit: int = 30):
        """

        :param x: x coord in image
        :param y: y coord in image
        """
        super().__init__(x, y)
        self.limit=limit

    def __str__(self):
        return f"SpeedSign @{self.x},{self.y}; limit:{self.limit}"