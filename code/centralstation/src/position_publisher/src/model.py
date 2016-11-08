import numpy as np


class Model(object):
    def __init__(self, coeffs):
        self.coeffs = coeffs

    def spaceToValue(self, distance):
        pass

    def derivative(self, distance):
        pass


class Poly3(Model):
    def __init__(self, coeffs):
        Model.__init__(self, coeffs)

    def spaceToValue(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        return self.coeffs[0] * (abs_distance ** 3) + self.coeffs[1] * (abs_distance ** 2) + self.coeffs[2] * abs_distance + self.coeffs[3]

    def derivative(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        dx = self.coeffs[0] * 1.5 * abs_distance * 2 * distance[0] + self.coeffs[1] * 2 * distance[0] + self.coeffs[2] * distance[0] / abs_distance
        dy = self.coeffs[0] * 1.5 * abs_distance * 2 * distance[1] + self.coeffs[1] * 2 * distance[1] + self.coeffs[2] * distance[1] / abs_distance
        return dx, dy


class Poly2(Model):
    def __init__(self, coeffs):
        Model.__init__(self, coeffs)

    def spaceToValue(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        return self.coeffs[0] * abs_distance ** 2 + self.coeffs[1] * abs_distance + self.coeffs[2]

    def derivative(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        dx = self.coeffs[0] * 2 * distance[0] + self.coeffs[1] * distance[0] * 1 / abs_distance
        dy = self.coeffs[0] * 2 * distance[1] + self.coeffs[1] * distance[1] * 1 / abs_distance
        return dx, dy

