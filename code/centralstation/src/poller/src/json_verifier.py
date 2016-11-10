import json


def containsMeasurements(data):
    if len(data) <= 0:
        return False

    beacons = json.loads(data)
    beacons = beacons["beacons"]
    return beacons != "ERR" and beacons != ""
