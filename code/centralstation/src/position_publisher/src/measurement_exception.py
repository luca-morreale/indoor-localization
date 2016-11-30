
class NoMeasurementException(Exception):
    def __init__(self, tag, text="No correspondance found for tag id "):
        super(Exception, self).__init__(text + str(tag))
