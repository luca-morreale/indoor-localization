import json


def extractJson(data):
	if len(data) <= 0:
		return {}

	beacons = json.loads(data)
	return beacons["beacons"]
