import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
import json
from ukf.matrix import Matrix
from ukf.measurement_package import MeasurementPackage
from ukf.ukf import UKF
from ukf.tools import Tools
from math import sin, cos, sqrt
import matplotlib.pyplot as plt
import sys, csv

sio = socketio.Server()
app = Flask(__name__)

# Create a Kalman Filter instance
ukf = UKF()

# used to compute the RMSE later
tools = Tools()
estimations = []
ground_truth = []
lidar_nis = []
radar_nis = []

def parse_measurement(row):
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


def run(ukf, p):
    ukf._std_a = p[0]
    ukf._std_yawdd = p[1]
    in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt"

    in_file = open(in_file_name_, newline='')
    data_reader = csv.reader(in_file, delimiter='\t', quotechar='|')


    tools = Tools()

    # used to compute the RMSE later
    estimations = []
    ground_truth =[]

    # prep the measurement packages (each line represents a measurement at a
    # timestamp)
    for row in data_reader:
        meas_package, gt_values =  parse_measurement(row)
        ground_truth.append(gt_values)

        ukf.process_measurement(meas_package)
        p_x = ukf._x.value[0][0]
        p_y = ukf._x.value[1][0]
        v   = ukf._x.value[2][0]
        yaw = ukf._x.value[3][0]

        v1 = cos(yaw)*v
        v2 = sin(yaw)*v

        estimate = Matrix([[p_x], [p_y], [v1], [v2]])

        estimations.append(estimate)

    # compute the accuracy (RMSE)
    rmse = tools.calculate_rmse(estimations, ground_truth)

    in_file.close()

    err = 0
    for i in rmse.value:
        err += i[0]*i[0]
    err = sqrt(err)
    return err

def twiddle(tol=0.02):
    p = [3, 1]
    dp = [0.5, 0.2]
    ukf = UKF()
    best_err = run(ukf, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            ukf = UKF()
            err = run(ukf, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                p[i] = max(0.1, p[i])
                ukf = UKF()
                err = run(ukf, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        j = data

        sensor_measurment = j["sensor_measurement"]

        tokens = sensor_measurment.split()

        i = 0
        # reads first element from the current line
        sensor_type = tokens[i]
        meas_package, gt_values =  parse_measurement(tokens)
        ground_truth.append(gt_values)

        #Call ProcessMeasurment(meas_package) for Kalman filter
        ukf.process_measurement(meas_package)

        #Push the current estimated x,y positon from the Kalman filter's state vector

        p_x = ukf._x.value[0][0]
        p_y = ukf._x.value[1][0]
        v  = ukf._x.value[2][0]
        yaw = ukf._x.value[3][0]

        v1 = cos(yaw)*v
        v2 = sin(yaw)*v

        estimate = Matrix([[p_x], [p_y], [v1], [v2]])

        estimations.append(estimate)
        if(sensor_type == "L"):
            lidar_nis.append(ukf._lidar_nis)
        elif(sensor_type == "R"):
            radar_nis.append(ukf._radar_nis)

        rmse = tools.calculate_rmse(estimations, ground_truth)

        msgJson = {}
        msgJson["estimate_x"] = p_x
        msgJson["estimate_y"] = p_y
        msgJson["rmse_x"] =  rmse.value[0][0]
        msgJson["rmse_y"] =  rmse.value[1][0]
        msgJson["rmse_vx"] = rmse.value[2][0]
        msgJson["rmse_vy"] = rmse.value[3][0]
        #msg = "42[\"estimate_marker\"," + msgJson.dump() + "]"
        # print(msg)
        sio.emit('estimate_marker', data=msgJson, skip_sid=True)
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

@sio.on('disconnect')
def disconnect(sid):
    if len(lidar_nis) > 0:
        plt.plot(lidar_nis, "r.")
        for i in range(len(lidar_nis)):
            lidar_nis[i] = 0.103
        plt.plot(lidar_nis, "y--")
        for i in range(len(lidar_nis)):
            lidar_nis[i] = 5.991
        plt.plot(lidar_nis, "y--")
        plt.title("Lidar NIS")
        plt.savefig("../images/lidar_nis.png")
        plt.clf()
        del lidar_nis[:]
    if len(radar_nis) > 0:
        plt.plot(radar_nis, "r.")
        for i in range(len(radar_nis)):
            radar_nis[i] = 0.352
        plt.plot(radar_nis, "y--")
        for i in range(len(radar_nis)):
            radar_nis[i] = 7.815
        plt.plot(radar_nis, "y--")
        plt.title("Radar NIS")
        plt.savefig("../images/radar_nis.png")
        plt.clf()
        del radar_nis[:]
    print("disconnected")

if __name__ == '__main__':
    if "--twiddle" in sys.argv:
        print("Final parameters: {}".format(twiddle()))
        sys.exit(0)
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
