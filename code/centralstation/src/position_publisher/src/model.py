import numpy as np


class Model(object):
    def __init__(self, coeffs):
        self.coeffs = coeffs

    def value(self, distance):
        pass

    def d(self, abs_distance, val):
        pass

    def derivative(self, distance):
        pass


class Poly3(Model):
    def __init__(self, coeffs):
        Model.__init__(self, coeffs)

    def value(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        return self.coeffs[0] * (abs_distance ** 3) + self.coeffs[1] * (abs_distance ** 2) + self.coeffs[2] * abs_distance + self.coeffs[3]

    def d(self, abs_distance, val):
        return self.coeffs[0] * 1.5 * abs_distance * 2 * val + self.coeffs[1] * 2 * val + self.coeffs[2] * val / abs_distance

    def derivative(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        dx = self.d(abs_distance, distance[0])
        dy = self.d(abs_distance, distance[1])
        return dx, dy


class Poly2(Model):
    def __init__(self, coeffs):
        Model.__init__(self, coeffs)

    def value(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        return self.coeffs[0] * abs_distance ** 2 + self.coeffs[1] * abs_distance + self.coeffs[2]

    def d(self, abs_distance, val):
        return self.coeffs[0] * 2 * val + self.coeffs[1] * val * 1 / abs_distance

    def derivative(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        dx = self.d(abs_distance, distance[0])
        dy = self.d(abs_distance, distance[1])
        return dx, dy


class Poly5(Model):
    def __init__(self, coeffs):
        Model.__init__(self, coeffs)

    def value(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        return self.coeffs[0] * (abs_distance ** 5) + self.coeffs[1] * (abs_distance ** 4) + self.coeffs[2] * (abs_distance ** 3) + \
                self.coeffs[3] * (abs_distance ** 2) + self.coeffs[4] * abs_distance + self.coeffs[5]

    def d(self, abs_distance, val):
        return self.coeffs[0] * 2.5 * (abs_distance ** 1.5) * 2 * val + self.coeffs[1] * 2 * (abs_distance) * 2 * val + \
                self.coeffs[2] * 1.5 * abs_distance * 2 * val + self.coeffs[3] * 2 * val + self.coeffs[4] * val / abs_distance

    def derivative(self, distance):
        quad_distance = np.dot(distance, distance)
        abs_distance = np.sqrt(quad_distance)
        dx = self.d(abs_distance, distance[0])
        dy = self.d(abs_distance, distance[1])
        return dx, dy
