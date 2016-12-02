
import operator
import numpy as np

import rospy
from std_msgs.msg import String

from poller.msg import MeasurementList

from matrix_operations import transpose, invert, multiply

class EKF(object):
    def __init__(self, tag, basestations, basestation_selector, model, var_z, max_station_selection):
        self.tag = tag
        self.model = model
        self.var_z = var_z
        self.last_update = -1
        self.initial_pool = {}
        self.init_pool_length = 0
        self.address_to_index = {}
        self.prediction_sequence = []
        self.basestations = basestations
        self.sensor_size = len(basestations)
        self.basestation_selector = basestation_selector
        self.max_station_selection = max_station_selection

        self.initMatrixes()
        self.createCommunicators()
        self.generateBasestationDictionary()

    def initMatrixes(self):
        self.createMatrixes(0.4)
        self.cov_matrix = self.Q

    ''' Creates observation covariance matrix, process noise covariance matrix and state transition model matrix
        ARGS:
            dt      distance in time wrt last update
    '''
    def createMatrixes(self, dt):
        self.estimated_position = np.zeros(4)
        # observation covariance
        self.R = np.eye(N=self.sensor_size, M=self.sensor_size) * self.var_z
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

    def createCommunicators(self):
        self.measurement_requester = rospy.Publisher('measurements_request', String, queue_size=10)
        self.measurement_reciver = rospy.Subscriber('measurements', MeasurementList, self.receiveMeasurements)

    def generateBasestationDictionary(self):
        for i in range(len(self.basestations)):
            self.address_to_index[self.basestations[i].address] = i

    def containsUsefulMeasurement(self, data):
        return any(str(pair.tag) == str(self.tag) for pair in data)

    ''' Returns the index of the given basestation address
        ARGS:
            basestation_address     string containing the address of the basestation
        Return an integer
    '''
    def indexOf(self, basestation_address):
        return self.address_to_index[basestation_address]

    def getMeasurementFromList(self, pair_list):
        for pair in pair_list:
            if str(pair.tag) == str(self.tag):
                return pair.measurement

    def getDataFromMeasurementList(self, msg):
        id_station = self.indexOf(msg.basestation)
        data = self.getMeasurementFromList(msg.data)
        return id_station, data

    ''' Update the estimated position using the data given in the measurements.
        ARGS:
            msg     ros topic message of type MeasurementList (look in poller package)
    '''
    def receiveMeasurements(self, msg):
        if self.containsUsefulMeasurement(msg.data):
            id_station, data = self.getDataFromMeasurementList(msg)
            self.updatePosition(data, id_station, rospy.Time.now())

    ''' Decide how update the position, initialize with a new one or perform a ekf iteration.
        ARGS:
            data            data received from the measurements.
            id_station      index of the basestation that performed the measurements.
            current_time    time of receiving the update.
    '''
    def updatePosition(self, data, id_station, current_time):
        if self.last_update == -1:
            self.addMeasurementToInitialPool(id_station, data)
            if self.init_pool_length >= self.max_station_selection:
                self.setInitialPositionFromPool(current_time)
        else:
            self.ekfIteration(data, id_station, current_time - self.last_update)
            self.updateTime(current_time)

    def updateTime(self, current_time):
        self.last_update = current_time

    def addMeasurementToInitialPool(self, id_station, data):
        self.init_pool_length += 1
        self.initial_pool[id_station] = data

    '''
        Initialize the position of the target at the position of the basestation that performed the highest measurements.
    '''
    def setInitialPositionFromPool(self, current_time):
        station_address = max(self.initial_pool.iteritems(), key=operator.itemgetter(1))[0]
        self.initializePosition(self.indexOf(station_address))
        self.updateTime(current_time)

    def initializePosition(self, basestation_index):
        self.estimated_position[0] = self.basestations[basestation_index].position[0]
        self.estimated_position[1] = self.basestations[basestation_index].position[1]

    ''' Performs an iteration (prediction + correction) of ekf.
        ARGS:
            measurement     array containing the measurements.
            id_station      index of the basestation that generated the measurements.
            dt              distance from last update.
    '''
    def ekfIteration(self, measurement, id_station, dt):
        self.createMatrixes(dt.to_sec())
        measurements = np.zeros(self.sensor_size)
        measurements[id_station] = measurement
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
                h[i] = self.model.spaceToValue(self.estimated_position[0:2] - self.basestations[i].position)

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
        self.prediction_sequence.append(transpose(self.estimated_position))

    ''' Generate the list of the possible basestation to poll for measurements.
        Returns the current estimated position.
    '''
    def pollingLoop(self):
        indexes = self.basestation_selector(self.basestations, self.cov_matrix, self.estimated_position, self.max_station_selection)
        for i in indexes:
            self.measurement_requester.publish(self.basestations[i].address)
        return self.estimated_position

    def setInitialPositionToCloserBasestation(self):
        for station in self.basestations:
            self.measurement_requester.publish(station.address)
        return self.estimated_position
        # a possibility is to poll all station and the with a flag start the ekf at second measurement
        # other possibilities? wait all measurements and the decide
