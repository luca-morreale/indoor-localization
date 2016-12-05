#! /usr/bin/env python

import rospy

from poller.msg import MeasurementList

from model import Poly3
from basestation import Basestation
from circumference_publisher import CircumferencePublisher

class MeasurementsDebugNode(object):
    def __init__(self):
        rospy.init_node('tag_position_publisher', anonymous=True)
        self.rate = rospy.Rate(0.001)
        self.tag_color = {}
        self.basestations = []
        self.address_to_index = {}
        self.extractParams()
        self.generateBasestationDictionary()
        self.debugger = CircumferencePublisher(Poly3([-0.0001769, 0.01364, -0.4367, 6.111]), self.tag_color)
        self.measurement_reciver = rospy.Subscriber('/measurements', MeasurementList, self.receiveMeasurements)

    def extractParams(self):
        self.alpha = rospy.get_param("/debug_node/alpha")
        colors = rospy.get_param("/debug_node/colors")
        stations = rospy.get_param("/localization_node/basestations")
        self.buildColorsFrom(colors)
        self.buildBasestationFrom(stations)
        self.generateBasestationDictionary()

    def buildColorsFrom(self, colors):
        for color in colors:
            self.tag_color[color[0]] = [rgb for rgb in color if not isinstance(rgb, str)]
            self.tag_color[color[0]].append(self.alpha)

    def buildBasestationFrom(self, stations):
        for station in stations:
            self.basestations.append(Basestation(station[0], float(station[1]), float(station[2])))

    def generateBasestationDictionary(self):
        for i in range(len(self.basestations)):
            self.address_to_index[self.basestations[i].address] = i

    def indexOf(self, basestation_address):
        return self.address_to_index[basestation_address]

    def receiveMeasurements(self, msg):
        position, namespace = self.extractStationInfo(msg)

        for pair in msg.data:
            self.debugger.publishShape(position, pair.measurement, pair.tag, namespace)

    def extractStationInfo(self, msg):
        id_station = self.indexOf(msg.basestation)
        position = self.basestations[id_station].position
        namespace = self.basestations[id_station].namespace

        return position, namespace

    def debugLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = MeasurementsDebugNode()
        node.debugLoop()
    except rospy.ROSInterruptException:
        pass
