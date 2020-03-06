import numpy as np

# scikit-kinematics==0.2.0 was the last version to support python 2
from skinematics.imus import kalman_quat


class Kalman(object):

    def __init__(self, rate=10):
        self._accelerometer_data = []
        self._gyro_data = []
        self._magnetometer_data = []
        self._rate = rate


    def update(self, gyro, accelerometer, magnetometer):
        self._accelerometer_data.append(accelerometer)
        self._gyro_data.append(gyro)
        self._magnetometer_data.append(magnetometer)

    def get_quaternion(self):
        result = kalman_quat(self._rate, np.array(self._accelerometer_data), np.array(self._gyro_data),
                             np.array(self._magnetometer_data))

        return result[0]
