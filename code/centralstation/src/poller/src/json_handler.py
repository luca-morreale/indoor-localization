import json


def extractJson(data):
    if len(data) <= 0:
        raise NoMeasurementException("")

    beacons = json.loads(data)
    return beacons["beacons"]


def extractFirstRSSI(data):
    if len(data) <= 0:
        raise NoMeasurementException("")

    beacons = json.loads(data)
    return beacons["beacons"][0]["rssi"]


def extractRSSIForTag(data, tag):
    beacons = extractJson(data)
    return rssiForTag(beacons, tag)


def rssiForTag(beacons, tag):
    if containsError(beacons):
        raise NoMeasurementException(tag)
    for el in enumerate(beacons):
        if el[1]["id_tag"] == str(tag):
            return el[1]["rssi"]
    raise NoMeasurementException(tag)


def containsMeasurements(data):
    if len(data) <= 0:
        return False

    beacons = json.loads(data)
    return not containsError(beacons["beacons"])


def containsError(beacons):
    return beacons == "ERR" or beacons == ""


class NoMeasurementException(Exception):
    def __init__(self, tag, text="No correspondance found for tag id "):
        super(Exception, self).__init__(text + str(tag))
