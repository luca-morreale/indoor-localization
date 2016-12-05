#!/usr/bin/env python

import rospy

from model import Poly3
from debugged_ekf import DebuggedEKF
from localization_node import LocalizationNode
from debugged_basestation import BasestationDebug
from basestation_selector import selectBestPositions
from circumference_publisher import CircumferencePublisher

class DebuggedLocalizationNode(LocalizationNode):

    def __init__(self):
        self.tag_color = {}
        self.extractColorForTag()

        LocalizationNode.__init__(self)
        self.ekf = DebuggedEKF(self.tag, self.basestations, selectBestPositions, Poly3(LocalizationNode.COEFFS), self.var_z, self.max_selection, self.debug)

    def extractColorForTag(self):
        self.alpha = rospy.get_param("/localization_node/alpha")
        colors = rospy.get_param("/localization_node/colors")
        self.buildColorsFrom(colors)

    def buildColorsFrom(self, colors):
        for color in colors:
            self.tag_color[int(color[0])] = [rgb for rgb in color if not isinstance(rgb, str)]
            self.tag_color[int(color[0])].append(self.alpha)

    def buildBasestation(self, stations):
        debugger = CircumferencePublisher(Poly3([-0.0001769, 0.01364, -0.4367, 6.111]), self.tag_color)
        for station in stations:
            self.basestations.append(BasestationDebug(station[0], float(station[1]), float(station[2]), debugger))


if __name__ == '__main__':
    try:
        node = DebuggedLocalizationNode()
        node.localizationLoop()
    except rospy.ROSInterruptException:
        pass
