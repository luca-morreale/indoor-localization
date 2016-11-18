#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, String, PointStamped

from poller.basestation import Basestation
from polller.json_handler import NoMeasurementException

from model import Poly3
from basestation_selector import BasestationSelector


class LocalizationNode(object):
    # model in meters
    COEFFS = [-0.1416, 2.311, -14.76, 36.79]

    EVERY_MINUTE = 0.1667
    EVERY_SECOND = 1

    def __init__(self):
        rospy.init_node('tag_position_publisher', anonymous=True)
        self.rate = rospy.Rate(LocalizationNode.EVERY_SECOND)
        self.basestations = []
        self.extractParams()

        self.tag = rospy.get_param("/localization_node/tag")
        self.frame = '/target_' + str(self.tag)
        self.ekf = EKF(self.tag, self.basestations, BasestationSelector(self.basestations), Poly3(LocalizationNode.COEFFS))
        self.publisher = rospy.Publisher(self.frame, PointStamped, queue_size=10)

    def extractParams(self):
        self.tag = rospy.get_param("/localization_node/tag")
        stations = rospy.get_param("/localization_node/basestations")
        self.buildBasestation(stations)

    def buildBasestation(self, stations):
        for station in stations:
            self.basestations.append(Basestation(station[0], float(station[1]), float(station[2])))

    def localizationLoop(self):
        while not rospy.is_shutdown():
            try:
                if self.miss_counter >= 2:  # after two misses it is assumed the target is not in range
                    self.rate = rospy.Rate(LocalizationNode.EVERY_MINUTE)  # once in a minute
                    position = self.ekf.setInitialPositionToCloserBasestation()
                else:
                    position = self.ekf.pollingLoop()

                self.publishPosition(position)
                self.rate = rospy.Rate(LocalizationNode.EVERY_SECOND)  # normal rate
                self.miss_counter = 0

            except NoMeasurementException:  # no measurements means the target is not in range
                self.miss_counter += 1

            self.rate.sleep()

    def publishPosition(self, position):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame

        msg.point = Point(position[0], position[1], 0)
        self.publisher.publish(msg)
        for basestation in self.basestations:
            basestation.publishPosition()


if __name__ == '__main__':
    try:
        node = LocalizationNode()
        node.localizationLoop()
    except rospy.ROSInterruptException:
        pass
