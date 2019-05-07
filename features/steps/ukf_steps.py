import sys

sys.path.append("./python")

from behave import *
from ukf.ukf import UKF
from ukf.tools import Tools
from helpers import *
from numpy.testing import assert_array_almost_equal, assert_array_less
from math import floor, log10, sin, cos
from unittest.mock import MagicMock, Mock

@given(u'a new UKF')
def step_impl(context):
    context._ukf = UKF()

    augmented_sigma_points_imp = context._ukf.augmented_sigma_points
    def call_and_save_augmented_sigma_points():
        context._vars["Xsig_aug"] = augmented_sigma_points_imp()
        return context._vars["Xsig_aug"]

    context._vars = {}
    context._ukf.augmented_sigma_points = MagicMock(side_effect=call_and_save_augmented_sigma_points)
    context._ukf.predict_mean_and_covariance = MagicMock(wraps=context._ukf.predict_mean_and_covariance)
    context._function_mapping = {"AugmentedSigmaPoints":"augmented_sigma_points",
                                 "PredictMeanAndCovariance":"predict_mean_and_covariance"}

    context._ukf_sequence = Mock()
    context._ukf_sequence.attach_mock(context._ukf.augmented_sigma_points, 'augmented_sigma_points')
    context._ukf_sequence.attach_mock(context._ukf.predict_mean_and_covariance, 'predict_mean_and_covariance')

@given(u'Dataset 1 is used')
def step_impl(context):
    context._measurements = []
    in_file_name_ = "./data/obj_pose-laser-radar-synthetic-input.txt"
    in_file = open(in_file_name_, "r")
    assert in_file.closed is False

    line = in_file.readline()

    while line:
        context._measurements.append(line)
        line = in_file.readline()

    # close files
    in_file.close()


@when(u'I run UnscentedKF')
def step_impl(context):
    context._ground_truth = []
    context._estimations = []
    for line in context._measurements:
        meas_package, gt_values = parse_measurement(line)
        context._ground_truth.append(gt_values)

        context._ukf.process_measurement(meas_package)
    
        # Push the current estimated x,y positon from the Kalman filter's 
        #   state vector

        p_x = context._ukf._x.value[0][0]
        p_y = context._ukf._x.value[1][0]
        v   = context._ukf._x.value[2][0]
        yaw = context._ukf._x.value[3][0]

        v1 = cos(yaw)*v
        v2 = sin(yaw)*v

        estimate = Matrix([[p_x], [p_y], [v1], [v2]])
        
        context._estimations.append(estimate)


@then(u'final rmse should be less than')
def step_impl(context):
    tools = Tools()
    expected_rmse = parse_vector(context.text)
    rmse = tools.calculate_rmse(context._estimations, context._ground_truth)
    assert_array_less(rmse.value, expected_rmse.value)


@then(u'is_initialized_ should be false')
def step_impl(context):
    assert context._ukf._is_initialized is False


@then(u'n_x_ should be 5')
def step_impl(context):
    assert context._ukf._n_x is 5


@then(u'n_aug_ should be 7')
def step_impl(context):
    assert context._ukf._n_aug is 7


@then(u'lambda_ should be -4')
def step_impl(context):
    assert context._ukf._lambda is -4


@then(u'weights_ should be of size 15')
def step_impl(context):
    assert context._ukf._weights.dimx is 15


@then(u'Xsig_pred_ should be of size 5 by 15')
def step_impl(context):
    assert context._ukf._Xsig_pred.dimx is 5
    assert context._ukf._Xsig_pred.dimy is 15


@given(u'Xsig_pred_ is')
def step_impl(context):
    context._ukf._Xsig_pred = parse_matrix(context.text)


@when(u'I call PredictMeanAndCovariance')
def step_impl(context):
    x, P = context._ukf.predict_mean_and_covariance()
    context._vars["x"] = x
    context._vars["P"] = P


@then(u'vector {variable} with accuracy {accuracy} should be')
def step_impl(context, variable, accuracy):
    expected_val = parse_vector(context.text)
    assert_array_almost_equal(context._vars[variable].value, expected_val.value, 
                              decimal=(int)(floor(log10(1.0/float(accuracy)))))


@then(u'matrix {variable} with accuracy {accuracy} should be')
def step_impl(context, variable, accuracy):
    expected_val = parse_matrix(context.text)
    assert_array_almost_equal(context._vars[variable].value, expected_val.value, 
                              decimal=(int)(floor(log10(1.0/float(accuracy)))))


@given(u'std_radr_ is {std_radr}')
def step_impl(context, std_radr):
    context._ukf._std_radr = float(std_radr)


@given(u'std_radphi_ is {std_radphi}')
def step_impl(context, std_radphi):
    context._ukf._std_radphi = float(std_radphi)


@given(u'std_radrd_ is {std_radrd}')
def step_impl(context, std_radrd):
    context._ukf._std_radrd = float(std_radrd)


@when(u'I call PredictRadarMeasurement')
def step_impl(context):
    Zsig, z_pred, S = context._ukf.predict_radar_measurement()
    context._vars["z_pred"] = z_pred
    context._vars["S"] = S


@given(u'std_a_ is {std_a}')
def step_impl(context, std_a):
    context._ukf._std_a = float(std_a)


@given(u'std_yawdd_ is {std_yawdd}')
def step_impl(context, std_yawdd):
    context._ukf._std_yawdd = float(std_yawdd)


@given(u'x_ is')
def step_impl(context):
    context._ukf._x = parse_vector(context.text)


@given(u'P_ is')
def step_impl(context):
    context._ukf._P = parse_matrix(context.text)


@when(u'I call Prediction with {delta_t}')
def step_impl(context, delta_t):
    context._ukf.prediction(float(delta_t))


@then(u'{method} should be called')
def step_impl(context, method):
    name, args, kwargs = context._ukf_sequence.mock_calls[0]
    assert name is context._function_mapping[method]
    context._call_args = args
    context._ukf_sequence.mock_calls = context._ukf_sequence.mock_calls[1:]


@then(u'Xsig_pred_ with accuracy {accuracy} should be')
def step_impl(context, accuracy):
    expected_Xsig_pred = parse_matrix(context.text)
    assert_array_almost_equal(context._ukf._Xsig_pred.value, expected_Xsig_pred.value, 
                              decimal=(int)(floor(log10(1.0/float(accuracy)))))


@given(u'{ref} measurement')
def step_impl(context, ref):
    context._meas_package, gt_values = parse_measurement(context.text)


@when(u'I call ProcessMeasurement')
def step_impl(context):
    context._ukf.process_measurement(context._meas_package)


@then(u'x_ with accuracy {accuracy} should be')
def step_impl(context, accuracy):
    expected_x = parse_vector(context.text)
    assert_array_almost_equal(context._ukf._x.value, expected_x.value, 
                              decimal=(int)(floor(log10(1.0/float(accuracy)))))


@then(u'P_ with accuracy {accuracy} should be')
def step_impl(context, accuracy):
    expected_P = parse_matrix(context.text)
    assert_array_almost_equal(context._ukf._P.value, expected_P.value, 
                              decimal=(int)(floor(log10(1.0/float(accuracy)))))


@given(u'I called ProcessMeasurement')
def step_impl(context):
    context._ukf.process_measurement(context._meas_package)


@given(u'Prediction would get called')
def step_impl(context):
    context._ukf.prediction = MagicMock(wraps=context._ukf.prediction)
    context._function_mapping["Prediction"]="prediction"

    context._ukf_sequence = Mock()
    context._ukf_sequence.attach_mock(context._ukf.prediction, 'prediction')


@given(u'UpdateLidar would get called')
def step_impl(context):
    context._ukf.update_lidar = MagicMock(wraps=context._ukf.update_lidar)
    context._function_mapping["UpdateLidar"]="update_lidar"
    context._ukf_sequence.attach_mock(context._ukf.update_lidar, 'update_lidar')


@then(u'It should have received delta_t as {expected_dt}')
def step_impl(context, expected_dt):
    assert context._call_args[0] == float(expected_dt)


@then(u'It should have received measurement in')
def step_impl(context):
    measurement, gt = parse_measurement(context.text)
    assert context._call_args[0] == measurement


@given(u'UpdateRadar would get called')
def step_impl(context):
    context._ukf.update_radar = MagicMock(wraps=context._ukf.update_radar)
    context._function_mapping["UpdateRadar"]="update_radar"
    context._ukf_sequence.attach_mock(context._ukf.update_radar, 'update_radar')


@given(u'PredictRadarMeasurement would get called')
def step_impl(context):
    def predict_radar_measurement_mock():
        return (context._vars["Zsig"], context._vars["z_pred"], context._vars["S"])

    context._ukf.predict_radar_measurement = MagicMock(side_effect=predict_radar_measurement_mock)
    context._function_mapping["PredictRadarMeasurement"]="predict_radar_measurement"

    context._ukf_sequence.attach_mock(context._ukf.predict_radar_measurement, 
                                      'predict_radar_measurement')


@given(u'It would return matrix {variable} as')
def step_impl(context, variable):
    return_val = parse_matrix(context.text)
    context._vars[variable] = return_val


@given(u'It would return vector {variable} as')
def step_impl(context, variable):
    return_val = parse_vector(context.text)
    context._vars[variable] = return_val


@when(u'I call UpdateRadar')
def step_impl(context):
    context._ukf.update_radar(context._meas_package)
