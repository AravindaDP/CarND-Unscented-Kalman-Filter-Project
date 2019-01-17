import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
import json
from ukf.matrix import Matrix
from ukf.measurement_package import MeasurementPackage
from ukf.ukf import UKF
from ukf.tools import Tools

sio = socketio.Server()
app = Flask(__name__)

# Create a Kalman Filter instance
ukf = UKF()

# used to compute the RMSE later
tools = Tools()
estimations = []
ground_truth = []

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        j = data

        sensor_measurment = j["sensor_measurement"]

        meas_package = MeasurementPackage()
        tokens = sensor_measurment.split()

        i = 0
        # reads first element from the current line
        sensor_type = tokens[i]
        i += 1

        if(sensor_type == "L"):
            meas_package._sensor_type = MeasurementPackage.SensorType.LASER

            px = float(tokens[i])
            py = float(tokens[i+1])
            meas_package._raw_measurements = Matrix([[px], [py]])
            timestamp = int(tokens[i+2])
            meas_package._timestamp = timestamp
            i += 3
        elif (sensor_type == "R"):
            meas_package._sensor_type = MeasurementPackage.SensorType.RADAR

            ro = float(tokens[i])
            theta = float(tokens[i+1])
            ro_dot = float(tokens[i+2])
            meas_package._raw_measurements = Matrix([[ro], [theta], [ro_dot]])
            timestamp = int(tokens[i+3])
            meas_package._timestamp = timestamp
            i += 4

        x_gt = float(tokens[i])
        y_gt = float(tokens[i+1])
        vx_gt = float(tokens[i+2])
        vy_gt = float(tokens[i+3])

        gt_values =  Matrix([[x_gt], [y_gt], [vx_gt], [vy_gt]])
        ground_truth.append(gt_values)

        #Call ProcessMeasurment(meas_package) for Kalman filter
        ukf.process_measurement(meas_package)

        #Push the current estimated x,y positon from the Kalman filter's state vector

        p_x = ukf._x.value[0][0]
        p_y = ukf._x.value[1][0]
        v1  = ukf._x.value[2][0]
        v2 = ukf._x.value[3][0]

        estimate = Matrix([[p_x], [p_y], [v1], [v2]])

        estimations.append(estimate)

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

if __name__ == '__main__':
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
