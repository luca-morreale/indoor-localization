#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

from model import Poly3
from basestation_selector import BasestationSelector
from ekf import EKF
from basestation import Basestation
from json_extractor import NoMeasurementException


class LocalizationNode(object):
    # model in meters
    COEFFS = [-0.1416, 2.311, -14.76, 36.79]

    EVERY_MINUTE = 0.1667
    EVERY_SECOND = 1

    def __init__(self):
        rospy.init_node('position_publisher', anonymous=True)
        self.publisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(LocalizationNode.EVERY_SECOND)

        self.extractParams()
        self.model = Poly3(LocalizationNode.COEFFS)
        self.selector = BasestationSelector(self.basestations)
        self.ekf = EKF(self.tag, 1, 0.1, self.basestations, self.model, self.selector)
        self.miss_counter = 2

    def extractParams(self):
        self.tag = rospy.get_param("/localization_node/tag")
        stations = rospy.get_param("/localization_node/basestations")
        self.buildBasestation(stations)

    def buildBasestation(self, stations):
        self.basestations = []
        for station in stations:
            self.basestations.append(Basestation(station[0], station[1], station[2]))

    def publishPosition(self, position):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/odom'
        msg.child_frame_id = '/target_' + str(self.tag)

        msg.pose.pose.position = Point(position[0], position[1], 0)
        self.publisher.publish(msg)

    def localizationLoop(self):
        while not rospy.is_shutdown():
            try:
                if self.miss_counter >= 2:  # after two misses it is assumed the target is not in range
                    self.rate = rospy.Rate(LocalizationNode.EVERY_MINUTE)  # once in a minute
                    position = self.ekf.setInitialPositionToCloserBasestation()
                else:
                    position = self.ekf.ekfIteration()

                self.publishPosition(position)
                self.rate = rospy.Rate(LocalizationNode.EVERY_SECOND)  # normal rate
                self.miss_counter = 0

            except NoMeasurementException:  # no measurements means the target is not in range
                self.miss_counter += 1

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = LocalizationNode()
        node.localizationLoop()
    except rospy.ROSInterruptException:
        pass
