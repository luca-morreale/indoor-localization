import numpy as np
from client import Client
from json_extractor import extractRSSIForTag
from matrix_operations import transpose, invert, multiply


class EKF(object):
    def __init__(self, tag, dt, var_z, beacons, model, selector):
        self.tag = tag
        self.dt = dt
        self.var_z = var_z
        self.sensor_size = len(beacons)
        self.beacons = beacons  # of type Basestation
        self.model = model
        self.selector = selector
        self.client = Client(10019)
        self.Ex = np.array([[dt ** 3 / 3, 0, dt ** 2 / 2, 0],
                            [0, dt ** 3 / 3, 0, dt ** 2 / 2],
                            [dt ** 2 / 2, 0, dt, 0],
                            [0, dt ** 2 / 2, 0, dt]])
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.initVariables()

    def initVariables(self):
        self.Ez = np.eye(N=self.sensor_size, M=self.sensor_size) * self.var_z
        self.cov_matrix = self.Ex
        self.prediction_sequence = []

    def setInitialPosition(self, init_pos):
        self.initVariables()
        self.estimated_position = np.zeros(4)
        for i in range(0, 2):
            self.estimated_position[i] = init_pos[i]
        self.prediction_sequence.append(self.estimated_position)

    def setInitialPositionToCloserBeacon(self):
        closer = self.getCloserBeacon()
        self.setInitialPosition(closer.position)
        return self.estimated_position

    def getCloserBeacon(self):
        measurements = self.getAllMeasurements()
        valid_beacons = self.selector.sortWithIndeces(measurements)
        return self.beacons[valid_beacons[0][0]]

    def getAllMeasurements(self):
        measurements = []
        for beacon in self.beacons:
            data = self.client.pollBasestation(beacon.address)
            measurements.append(extractRSSIForTag(data, self.tag))
        return measurements

    def ekf(self):
        H, h, measurements, estimated_cov_matrix = self.prediction()
        self.correction(H, h, measurements, estimated_cov_matrix)
        return self.estimated_position

    def prediction(self):
        self.estimated_position = multiply(self.F, self.estimated_position)
        estimated_cov_matrix = multiply(self.F, self.cov_matrix, transpose(self.F)) + self.Ex
        measurements = self.selectiveMeasurements(estimated_cov_matrix)

        h = np.zeros(self.sensor_size)
        for i in range(0, self.sensor_size):
            if measurements[i] != 0:
                h[i] = self.model.spaceToValue(self.estimated_position[0] - self.beacons[i].position)

        # H is the measurement equation
        H = np.empty((0, 4))
        for i in range(0, len(measurements)):
            if measurements[i] != 0:
                dh_dx, dh_dy = self.model.derivative(self.estimated_position[0] - self.beacons[i].position)
            else:
                dh_dx, dh_dy = 0.0, 0.0
            H = np.append(H, np.array([[dh_dx, dh_dy, 0.0, 0.0]]), axis=0)

        return H, h, measurements, estimated_cov_matrix

    def correction(self, H, h, measurements, estimated_cov_matrix):
        H_hat = multiply(H, estimated_cov_matrix, transpose(H))
        K = multiply(estimated_cov_matrix, transpose(H), invert(H_hat + self.Ez))

        self.estimated_position += multiply(K, (measurements - h))
        self.cov_matrix = multiply((np.eye(N=4) - multiply(K, H)), estimated_cov_matrix)
        self.prediction_sequence.append(transpose(self.estimated_position))

    def selectiveMeasurements(self, estimated_cov_matrix):
        indexes = self.selector.selectBestPositions(estimated_cov_matrix, self.estimated_position)

        measurements = np.zeros(self.sensor_size)
        for i in indexes:
            data = self.client.pollBasestation(self.beacons[i].address)
            measurements[i] = extractRSSIForTag(data, self.tag)

        return measurements
