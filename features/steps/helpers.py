import sys

sys.path.append("./python")

from ukf.matrix import Matrix
from ukf.measurement_package import MeasurementPackage

def parse_vector(line):
    return Matrix([[float(i)] for i in line.split()])

def parse_matrix(text):
    return Matrix([[float(i) for i in line.split()] for line in text.splitlines()])

def parse_measurement(line):
    row = line.split()
    meas_package = MeasurementPackage()

    i = 0
    # reads first element from the current line
    sensor_type = row[i]
    i += 1

    if(sensor_type == "L"):
        # LASER MEASUREMENT

        # read measurements at this timestamp
        meas_package._sensor_type = MeasurementPackage.SensorType.LASER

        px = float(row[i])
        py = float(row[i+1])
        meas_package._raw_measurements = Matrix([[px], [py]])
        timestamp = int(row[i+2])
        meas_package._timestamp = timestamp
        i += 3
    elif (sensor_type == "R"):
        # RADAR MEASUREMENT

        # read measurements at this timestamp
        meas_package._sensor_type = MeasurementPackage.SensorType.RADAR

        ro = float(row[i])
        theta = float(row[i+1])
        ro_dot = float(row[i+2])
        meas_package._raw_measurements = Matrix([[ro], [theta], [ro_dot]])
        timestamp = int(row[i+3])
        meas_package._timestamp = timestamp
        i += 4

    # read ground truth data to compare later
    x_gt = float(row[i])
    y_gt = float(row[i+1])
    vx_gt = float(row[i+2])
    vy_gt = float(row[i+3])

    gt_values =  Matrix([[x_gt], [y_gt], [vx_gt], [vy_gt]])

    return (meas_package, gt_values)

