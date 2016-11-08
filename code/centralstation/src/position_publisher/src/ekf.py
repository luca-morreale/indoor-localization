import numpy as np
from client import Client
from json_extractor import extractRSSIForTag
from json_extractor import NoMeasurementException
from matrix_operations import transpose, invert, multiply


class EKF(object):
    def __init__(self, tag, dt, var_z, basestations, model, selector):
        self.tag = tag
        self.dt = dt
        self.var_z = var_z
        self.sensor_size = len(basestations)
        self.basestations = basestations  # of type Basestation
        self.model = model
        self.selector = selector
        self.client = Client(10019)
        self.Ex = np.array([[dt ** 3 / 3,  0,            dt ** 2 / 2,  0],
                            [0,            dt ** 3 / 3,  0,            dt ** 2 / 2],
                            [dt ** 2 / 2,  0,            dt,           0],
                            [0,            dt ** 2 / 2,  0,            dt]])
        self.F = np.array([[1,  0,  dt,  0],
                           [0,  1,  0,  dt],
                           [0,  0,  1,  0],
                           [0,  0,  0,  1]])
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

    def setInitialPositionToCloserBasestation(self):
        closest = self.getCloserBasestation()
        self.setInitialPosition(closest.position)
        return self.estimated_position

    def getCloserBasestation(self):
        measurements = self.getAllMeasurements()
        valid_basestations = self.selector.sortWithIndeces(measurements)
        return self.basestations[valid_basestations[0][0]]

    def getAllMeasurements(self):
        return self.getMeasurementsForRange(range(0, self.sensor_size))

    def getMeasurementsForRange(self, indexes):
        measurements = np.zeros(self.sensor_size)
        for i in indexes:
            self.getMeasurement(i, measurements)

        if self.emptyMeasurements(measurements):
            raise NoMeasurementException(self.tag)
        return measurements

    def getMeasurement(self, index, array):
        try:
            data = self.client.pollBasestation(self.basestations[index].address)
            array[index] = extractRSSIForTag(data, self.tag)
        except NoMeasurementException:
            pass

    def emptyMeasurements(self, measurements):
        empty = np.zeros(self.sensor_size)
        return np.all(empty == measurements)

    def ekfIteration(self):
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
                h[i] = self.model.spaceToValue(self.estimated_position[0:2] - self.basestations[i].position)

        H = np.empty((0, 4))
        for i in range(0, len(measurements)):
            if measurements[i] != 0:
                dh_dx, dh_dy = self.model.derivative(self.estimated_position[0:2] - self.basestations[i].position)
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
        return self.getMeasurementsForRange(indexes)
