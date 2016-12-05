
from ekf import EKF


class DebuggedEKF(EKF):

    def updatePosition(self, data, id_station, current_time):
        EKF.updatePosition(self, data, id_station, current_time)
        self.basestations[id_station].publishMarker(data, self.tag)
