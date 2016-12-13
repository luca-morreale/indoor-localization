
import numpy as np

from matrix_operations import transpose, invert, multiply


class EKF(object):
    def __init__(self, basestations, basestation_selector, model, measurement_variance, measurement_weight, max_station_selection):
        self.model = model
        self.measurement_variance = measurement_variance
        self.measurement_weight = measurement_weight
        self.basestations = basestations
        self.sensor_size = len(basestations)
        self.basestation_selector = basestation_selector
        self.max_station_selection = max_station_selection

        self.initMatrixes()

    def initMatrixes(self):
        self.estimated_position = np.zeros(4)
        self.createMatrixes(0.4, 31)
        self.cov_matrix = self.Q

    ''' Creates observation covariance matrix, process noise covariance matrix and state transition model matrix
        ARGS:
            dt      distance in time wrt last update
    '''
    def createMatrixes(self, dt, measurement):
        # observation covariance
        self.R = np.eye(N=self.sensor_size, M=self.sensor_size) * self.measurement_variance
        # process noise covariance
        self.Q = np.array([[dt ** 3 / 3, 0, dt ** 2 / 2, 0],
                           [0, dt ** 3 / 3, 0, dt ** 2 / 2],
                           [dt ** 2 / 2, 0, dt, 0],
                           [0, dt ** 2 / 2, 0, dt]])
        # state transition model
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    ''' Performs an iteration (prediction + correction) of ekf.
        ARGS:
            measurement     array containing the measurements.
            id_station      index of the basestation that generated the measurements.
            dt              distance from last update.
    '''
    def ekfIteration(self, measurement, id_station, dt):
        self.createMatrixes(dt.to_sec(), measurement)
        measurements = np.zeros(self.sensor_size)
        measurements[id_station] = measurement - self.measurement_weight.value(measurement)
        H, h, estimated_cov_matrix = self.prediction(measurements)
        self.correction(H, h, measurements, estimated_cov_matrix)

    ''' Predicts the possible next state given the measurements.
        ARGS:
            measurements    array containing measurements
        Returns:
            h                       predicted measurement
            H                       observation model
            estimated_cov_matrix    estimated covariance matrix
    '''
    def prediction(self, measurements):
        self.estimated_position = multiply(self.F, self.estimated_position)
        estimated_cov_matrix = multiply(self.F, self.cov_matrix, transpose(self.F)) + self.Q

        h = np.zeros(self.sensor_size)
        for i in range(0, self.sensor_size):
            if measurements[i] != 0:
                h[i] = self.model.value(self.estimated_position[0:2] - self.basestations[i].position)

        H = np.empty((0, 4))
        for i in range(0, len(measurements)):
            if measurements[i] != 0:
                dh_dx, dh_dy = self.model.derivative(self.estimated_position[0:2] - self.basestations[i].position)
            else:
                dh_dx, dh_dy = 0.0, 0.0
            H = np.append(H, np.array([[dh_dx, dh_dy, 0.0, 0.0]]), axis=0)

        return H, h, estimated_cov_matrix

    ''' Performs a correction step of the position (and associated covariance matrix) given the parameters.
        ARGS:
            H                       observation model
            h                       measurements generated wrt estimated current position
            measurements            array of measurements obtained
            estimated_cov_matrix    current estimated covariance matrix
    '''
    def correction(self, H, h, measurements, estimated_cov_matrix):
        measurement_residual = measurements - h
        residual_covariance = multiply(H, estimated_cov_matrix, transpose(H)) + self.R
        kalman_gain = multiply(estimated_cov_matrix, transpose(H), invert(residual_covariance))

        self.estimated_position = self.estimated_position + multiply(kalman_gain, measurement_residual)
        self.cov_matrix = multiply((np.eye(N=4) - multiply(kalman_gain, H)), estimated_cov_matrix)
