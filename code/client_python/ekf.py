import numpy as np
from client import Client
from json_extractor import extractJson


class EKF():

    def __init__(self,dt,sensor_size,var_z,beacons,coeff):
        self.client = Client(10019)
        self.coeff = coeff
        self.beacons = beacons  # of type Basestation
        self.dt = dt
        self.sensor_size = sensor_size
        self.accel_noise_mag = 0.001**2/dt
        self.Ex = np.array([[dt**3/3, 0,       dt**2/2, 0],
                            [0,       dt**3/3, 0,       dt**2/2],
                            [dt**2/2, 0,       dt,      0],
                            [0,       dt**2/2, 0,       dt]])
        self.F = np.array([[1, 0, dt, 0],
                            [0, 1, 0, dt], 
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        self.x_hat = np.empty((0, 4)) #shape is (1,4) - to init with initial position
        self.Ez = np.eye(N=sensor_size, M=sensor_size) * var_z
        self.P = self.Ex
        self.prediction = np.transpose(self.x_hat)


    def ekf(self):
        H, h, measurements, P_hat = self.prediction()
        self.correction(H, h, measurements, P_hat)


    def prediction(self):
        self.x_hat = self.F * self.x_hat
        P_hat = self.F * self.P * np.transpose(self.F) + self.Ex
        measurements = self.selective_extraction(P_hat)
        h = np.zeros(self.sensor_size)
        for i in range(0, self.sensor_size):
            if (measurements[i] != 0):
                #h[i] = distance(self.x_hat[0],self.beacons[i])
                h[i] = self.space_to_value(self.x_hat[0] - self.beacons[i].position)

        #H is the measurement equation
        H = np.empty((0, 4))
        for i in range(0, len(measurements)):
            if (measurements[i] != 0):
                dh_dx, dh_dy = self.derivative(self.x_hat[0] - self.beacons[i].position)
            else:
                dh_dx = 0.0
                dh_dy = 0.0
            H = np.append(H, np.array([[dh_dx, dh_dy, 0.0, 0.0]]), axis=0)

        return H, h, measurements, P_hat


    def correction(self, H, h, measurements, P_hat):
        K = P_hat * np.transpose(H) * np.invert(H * P_hat * np.transpose(H) + self.Ez)
        self.x_hat += K * (measurements - h)
        self.P = (np.eye(N=4) - K * H) * P_hat
        self.prediction = np.append(self.prediction, np.array([np.transpose(self.x_hat)]), axis=0)


    def selective_extraction(self, P_hat):
        indexes = self.position_selector(P_hat)
        measurements = []
        for i in indexes:
            data = self.client.pollBasestation(self.beacons[i].address)
            data = extractFirstRSSI(data)
            measurements += [data]
        return measurements


    def position_selector(self,P_hat):
        P_hat_inv = np.invert(P_hat)
        distances = []
        for i in self.sensor_size:
            b_position = np.append(self.beacons[i].position, np.array([0,0]));
            A = np.transpose(self.x_hat) - b_position
            distances += A * (P_hat_inv * np.transpose(A))

        sorted_tuples = sorted(enumerate(distances), key=lambda x: x[1])
        valid_distances = [item for item in sorted_tuples if item[1] > 0]
        return [valid_distances[i][0] for i in range(0,min(3,len(valid_distances)))]


    def derivative(self, arr):
        dx = self.coeff[0] * 1.5 * np.sqrt(arr * arr) * 2 * arr[0] + self.coeff[1] * 2 * arr[0] + self.coeff[2] * arr[
            0] * 1 / np.sqrt(arr * arr)
        dy = self.coeff[0] * 1.5 * np.sqrt(arr * arr) * 2 * arr[1] + self.coeff[1] * 2 * arr[1] + self.coeff[2] * arr[
            1] * 1 / np.sqrt(arr * arr)
        return dx,dy


    def space_to_value(self, arr):
        return self.coeff[0] * (np.sqrt(arr*arr))**3 + self.coeff[1] * (arr * arr) + self.coeff[2] * np.sqrt(arr * arr) + self.coeff[4]
