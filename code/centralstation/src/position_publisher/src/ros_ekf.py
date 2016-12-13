
import operator
import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped

from ekf import EKF
from poller.msg import MeasurementList
from position_covariance import CovariancePublisher


class ROSEKF(EKF):

    def __init__(self, tag, basestations, basestation_selector, model, measurement_variance, measurement_weight, max_station_selection, debug=False):
        EKF.__init__(self, basestations, basestation_selector, model, measurement_variance, measurement_weight, max_station_selection)
        self.tag = tag
        self.debug = debug
        self.last_update = -1
        self.initial_pool = {}
        self.init_pool_length = 0
        self.address_to_index = {}

        self.createCommunicators()
        self.generateBasestationDictionary()
        self.covariance_publisher = CovariancePublisher()

    def createCommunicators(self):
        self.measurement_requester = rospy.Publisher('measurements_request', String, queue_size=10)
        self.measurement_reciver = rospy.Subscriber('measurements', MeasurementList, self.receiveMeasurements)
        self.position_publisher = rospy.Publisher('/target_' + str(self.tag), PointStamped, queue_size=10)

    def generateBasestationDictionary(self):
        for i in range(len(self.basestations)):
            self.address_to_index[self.basestations[i].address] = i

    def debug_msg(self, msg):
        if self.debug:
            print msg

    '''
        Checks if contains measurement regarding the tag
    '''
    def containsUsefulMeasurement(self, data, station_address):
        return any(str(pair.tag) == str(self.tag) for pair in data) \
                    and station_address in self.address_to_index.keys()

    '''
        Returns the index of the given basestation address
    '''
    def indexOf(self, basestation_address):
        return self.address_to_index[basestation_address]

    '''
        Extracts the measurement for the tag from the given list
    '''
    def getMeasurementFromList(self, pair_list):
        for pair in pair_list:
            if str(pair.tag) == str(self.tag):
                return pair.measurement

    ''' Update the estimated position using the data given in the measurements.
        ARGS:
            msg     ros topic message of type MeasurementList (look in poller package)
    '''
    def receiveMeasurements(self, msg):
        self.debug_msg('Arrived a new message from' + str(msg.basestation))
        if self.containsUsefulMeasurement(msg.data, msg.basestation):
            id_station = self.indexOf(msg.basestation)
            data = self.getMeasurementFromList(msg.data)
            self.updatePosition(data, id_station, rospy.Time.now())
            self.debug_msg('contains usefull: ' + str(data))
            self.publishPosition()

    '''
        Publishes the estimated position of the target
    '''
    def publishPosition(self):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/target_' + str(self.tag)

        msg.point = Point(self.estimated_position[0], self.estimated_position[1], 0)
        self.position_publisher.publish(msg)
        self.publishCovarianceMatrix()

    def publishCovarianceMatrix(self):
        if self.debug:
            s = 5.991   # 95 % confidence
            rx = self.cov_matrix[0][0] * math.sqrt(s)
            ry = self.cov_matrix[1][1] * math.sqrt(s)

            self.covariance_publisher.publishShape(self.estimated_position, rx, ry, self.tag, 'covariance_debug')

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
        Initializes the position of the target at the position of the basestation that performed the highest measurements.
    '''
    def setInitialPositionFromPool(self, current_time):
        station_index = max(self.initial_pool.iteritems(), key=operator.itemgetter(1))[0]
        self.estimated_position[0] = self.basestations[station_index].position[0]
        self.estimated_position[1] = self.basestations[station_index].position[1]
        self.updateTime(current_time)

    ''' Generate the list of the possible basestation to poll for measurements.
        Returns the current estimated position.
    '''
    def pollingLoop(self):
        indexes = self.basestation_selector(self.basestations, self.cov_matrix, self.estimated_position, self.max_station_selection)
        for i in indexes:
            self.measurement_requester.publish(self.basestations[i].address)
        return self.estimated_position
