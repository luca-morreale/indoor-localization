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
        self.alpha = rospy.get_param("/localization_node/alpha")
        self.tag_color = {2022: [0, 1, 0]}
        self.extractColorForTag()

        LocalizationNode.__init__(self)
        self.ekf = DebuggedEKF(self.tag, self.basestations, selectBestPositions, Poly3(LocalizationNode.COEFFS), self.var_z, self.max_selection, self.debug)


    def extractColorForTag(self):
        pass


    def buildBasestation(self, stations):
        debugger = CircumferencePublisher(Poly3([-0.0001769, 0.01364, -0.4367, 6.111]), self.tag_color, self.alpha)
        for station in stations:
            self.basestations.append(BasestationDebug(station[0], float(station[1]), float(station[2]), debugger))


if __name__ == '__main__':
    try:
        node = DebuggedLocalizationNode()
        node.localizationLoop()
    except rospy.ROSInterruptException:
        pass
