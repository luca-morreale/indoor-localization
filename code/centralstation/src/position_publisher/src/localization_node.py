#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PointStamped

from ros_ekf import ROSEKF
from model import Poly3, Poly5
from basestation import Basestation
from basestation_selector import selectBestPositions


class LocalizationNode(object):

    EVERY_MINUTE = 0.1667
    EVERY_SECOND = 1
    EVERY_THREE_SECONDS = 3

    def __init__(self):
        rospy.init_node('tag_position_publisher', anonymous=True)
        self.rate = rospy.Rate(LocalizationNode.EVERY_SECOND)
        self.basestations = []
        self.extractParams()
        self.frame = '/target_' + str(self.tag)
        self.ekf = ROSEKF(self.tag, self.basestations, selectBestPositions, Poly3(self.measurement_model_coeffs),
                          self.var_z, Poly5(self.measurement_weight_model_coeffs), self.max_selection, self.debug)
        self.publisher = rospy.Publisher(self.frame, PointStamped, queue_size=10)

    def extractParams(self):
        self.tag = rospy.get_param("/localization_node/tag")
        self.var_z = rospy.get_param("/localization_node/var_z")
        self.debug = rospy.get_param("/localization_node/debug")
        self.max_selection = rospy.get_param("/localization_node/max_selection")
        self.measurement_model_coeffs = rospy.get_param("/localization_node/measurement_model")
        self.measurement_weight_model_coeffs = rospy.get_param("/localization_node/measurement_weight_model")
        stations = rospy.get_param("/localization_node/basestations")
        self.buildBasestation(stations)

    def buildBasestation(self, stations):
        for station in stations:
            self.basestations.append(Basestation(station[0], float(station[1]), float(station[2])))

    def localizationLoop(self):
        while not rospy.is_shutdown():
            self.ekf.pollingLoop()
            self.publishBasestationPosition()
            self.rate.sleep()

    def publishBasestationPosition(self):
        for basestation in self.basestations:
            basestation.publishPosition()


if __name__ == '__main__':
    try:
        node = LocalizationNode()
        node.localizationLoop()
    except rospy.ROSInterruptException:
        pass
