
import numpy as np
from matrix_operations import transpose, invert, multiply

def selectBestPositions(sensors, estimated_cov_matrix, estimated_position, size):
    sensor_size = len(sensors)
    size = sensor_size if size < 0 else size
    inv_estimated_cov_matrix = invert(estimated_cov_matrix)
    distances = np.ones(sensor_size) * -1
    extended_sensors_pos = extendSensorPosition(sensors)
    for i in range(sensor_size):
        difference = transpose(estimated_position) - extended_sensors_pos[i]
        distances[i] = multiply(difference, inv_estimated_cov_matrix, transpose(difference))

    valid_distances = sortWithIndeces(distances)
    return [valid_distances[i][0] for i in range(0, min(size, len(valid_distances)))]

def extendSensorPosition(sensors):
    extended_sensors = []
    for sensor in sensors:
        extended_sensors.append(np.append(sensor.position, np.array([0, 0])))
    return extended_sensors

def sortWithIndeces(list):
    sorted_tuples = sorted(enumerate(list), key=lambda x: x[1])
    return [item for item in sorted_tuples if item[1] > 0]
