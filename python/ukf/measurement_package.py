from ukf.matrix import Matrix
from enum import Enum

class MeasurementPackage :
    class SensorType(Enum):
        LASER = 1
        RADAR = 2

    def __init__(self, timestamp = 0, sensor_type = SensorType.LASER,
                 raw_measurements = Matrix([[]])):
        self._timestamp =  timestamp
        self._sensor_type = sensor_type
        self._raw_measurements = raw_measurements