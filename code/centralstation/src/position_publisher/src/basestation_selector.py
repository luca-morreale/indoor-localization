
import numpy as np
from matrix_operations import transpose, invert, multiply


class BasestationSelector(object):
    def __init__(self, basestations):
        self.basestations = basestations
        self.sensor_size = len(basestations)

    def selectBestPositions(self, estimated_cov_matrix, estimated_position, size=5):
        estimated_cov_matrix_inv = invert(estimated_cov_matrix)
        distances = np.ones(self.sensor_size) * -1
        for i in range(self.sensor_size):
            extended_basestation_pos = np.append(self.basestations[i].position, np.array([0, 0]))
            difference = transpose(estimated_position) - extended_basestation_pos
            distances[i] = multiply(difference, estimated_cov_matrix_inv, transpose(difference))

        valid_distances = self.sortWithIndeces(distances)
        return [valid_distances[i][0] for i in range(0, min(size, len(valid_distances)))]

    def sortWithIndeces(self, list):
        sorted_tuples = sorted(enumerate(list), key=lambda x: x[1])
        return [item for item in sorted_tuples if item[1] > 0]
