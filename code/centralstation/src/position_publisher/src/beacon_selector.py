import numpy as np
from matrix_operations import transpose, invert, multiply


class BeaconSelector(object):
    def __init__(self, beacons):
        self.beacons = beacons
        self.sensor_size = len(beacons)

    def selectBestPositions(self, estimated_cov_matrix, estimated_position):
        estimated_cov_matrix_inv = invert(estimated_cov_matrix)
        distances = np.ones(self.sensor_size) * -1
        for i in range(self.sensor_size):
            extended_beacon_pos = np.append(self.beacons[i].position, np.array([0, 0]))
            difference = transpose(estimated_position) - extended_beacon_pos
            distances[i] = multiply(difference, estimated_cov_matrix_inv, transpose(difference))

        valid_distances = self.sortWithIndeces(distances)
        return [valid_distances[i][0] for i in range(0, min(3, len(valid_distances)))]

    def sortWithIndeces(self, list):
        sorted_tuples = sorted(enumerate(list), key=lambda x: x[1])
        return [item for item in sorted_tuples if item[1] > 0]
