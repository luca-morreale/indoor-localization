import numpy as np
from client import Client
from json_extractor import extractRSSIForTag


class EKF(object):

    def __init__(self, tag, dt, sensor_size, var_z, beacons, coeff):
        self.tag = tag
        self.dt = dt
        self.sensor_size = sensor_size
        self.beacons = beacons  # of type Basestation
        self.model_coeffs = coeff
        self.client = Client(10019)
        self.Ex = np.array([[dt**3/3, 0,       dt**2/2, 0],
                            [0,       dt**3/3, 0,       dt**2/2],
                            [dt**2/2, 0,       dt,      0],
                            [0,       dt**2/2, 0,       dt]])
        self.F = np.array([[1, 0, dt, 0],
                            [0, 1, 0, dt], 
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        self.estimated_position = np.empty((0, 4))  # shape is (1,4) - to init with initial position
        self.Ez = np.eye(N=sensor_size, M=sensor_size) * var_z
        self.cov_matrix = self.Ex
        self.prediction_sequence = [np.transpose(self.estimated_position)]

    def ekf(self):
        H, h, measurements, estimated_cov_matrix = self.prediction()
        self.correction(H, h, measurements, estimated_cov_matrix)
        return self.estimated_position

    def prediction(self):
        self.estimated_position = self.F * self.estimated_position
        estimated_cov_matrix = self.F * self.cov_matrix * np.transpose(self.F) + self.Ex
        measurements = self.selectiveExtraction(estimated_cov_matrix)

        h = np.zeros(self.sensor_size)
        for i in range(0, self.sensor_size):
            if measurements[i] != 0:
                h[i] = self.space_to_value(self.estimated_position[0] - self.beacons[i].position)

        # H is the measurement equation
        H = np.empty((0, 4))
        for i in range(0, len(measurements)):
            if measurements[i] != 0:
                dh_dx, dh_dy = self.derivative(self.estimated_position[0] - self.beacons[i].position)
            else:
                dh_dx, dh_dy = 0.0, 0.0
            H = np.append(H, np.array([[dh_dx, dh_dy, 0.0, 0.0]]), axis=0)

        return H, h, measurements, estimated_cov_matrix

    def correction(self, H, h, measurements, estimated_cov_matrix):
        K = estimated_cov_matrix * np.transpose(H) * np.invert(H * estimated_cov_matrix * np.transpose(H) + self.Ez)
        self.estimated_position += K * (measurements - h)
        self.cov_matrix = (np.eye(N=4) - K * H) * estimated_cov_matrix
        self.prediction_sequence.append(np.transpose(self.estimated_position))

    def selectiveExtraction(self, estimated_cov_matrix):
        indexes = self.selectPosition(estimated_cov_matrix)
        measurements = []
        for i in indexes:
            data = self.client.pollBasestation(self.beacons[i].address)
            measurements += [extractRSSIForTag(self.tag, data)]
        return measurements

    def selectPosition(self, estimated_cov_matrix):
        estimated_cov_matrix_inv = np.invert(estimated_cov_matrix)
        distances = []
        for i in self.sensor_size:
            extended_beacon_pos = np.append(self.beacons[i].position, np.array([0, 0]))
            difference = np.transpose(self.estimated_position) - extended_beacon_pos
            distances += difference * (estimated_cov_matrix_inv * np.transpose(difference))

        valid_distances = self.sortWithIndeces(distances)
        return [valid_distances[i][0] for i in range(0, min(3, len(valid_distances)))]

    def derivative(self, arr):
        dx = self.model_coeffs[0] * 1.5 * np.sqrt(arr * arr) * 2 * arr[0] + self.model_coeffs[1] * 2 * arr[0] + \
                    self.model_coeffs[2] * arr[0] * 1 / np.sqrt(arr * arr)
        dy = self.model_coeffs[0] * 1.5 * np.sqrt(arr * arr) * 2 * arr[1] + self.model_coeffs[1] * 2 * arr[1] + \
                    self.model_coeffs[2] * arr[1] * 1 / np.sqrt(arr * arr)
        return dx, dy

    def space_to_value(self, arr):
        return self.model_coeffs[0] * (np.sqrt(arr * arr)) ** 3 + self.model_coeffs[1] * (arr * arr) + \
                    self.model_coeffs[2] * np.sqrt(arr * arr) + self.model_coeffs[4]

    def sortWithIndeces(self, list):
        sorted_tuples = sorted(enumerate(list), key=lambda x: x[1])
        return [item for item in sorted_tuples if item[1] > 0]
