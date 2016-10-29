import numpy as np


def invert(matrix):
    if len(matrix[0]) == 1:
        return 1 / matrix[0][0]
    return np.linalg.inv(matrix)


def multiply(*matrixs):
    result = matrixs[0]
    size = len(matrixs)
    for i in range(1, size):
        result = np.dot(result, matrixs[i])
    return result


def transpose(matrix):
    return np.transpose(matrix)