
from basestation import Basestation

class BasestationDebug(Basestation):
    def __init__(self, address, x, y, debugger):
        Basestation.__init__(self, address, x, y)
        self.debugger = debugger

    def publishMarker(self, measurement, tag):
        self.debugger.publish_shape(self.position, measurement, tag, self.custom_frame)
