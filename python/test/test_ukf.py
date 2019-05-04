import unittest
from unittest.mock import MagicMock, Mock, call
from ukf.ukf import UKF
from ukf.tools import Tools
from ukf.matrix import Matrix
from ukf.measurement_package import MeasurementPackage
from numpy.testing import assert_array_almost_equal, assert_array_less
import sys, os, csv
from math import sin, cos

class TestUKF(unittest.TestCase):
    def setUp(self):
        self._ukf = UKF()
        sys.stdout = open(os.devnull, 'w')

    def tearDown(self):
        sys.stdout.close()
        sys.stdout = sys.__stdout__

    def test_Constructor_SetsParametersAndMatrices(self):
        self.assertFalse(self._ukf._is_initialized)
        self.assertEqual(self._ukf._n_x, 5)
        self.assertEqual(self._ukf._n_aug, 7)
        self.assertEqual(self._ukf._lambda, 3-7)
        self.assertEqual(self._ukf._weights.dimx, 15)
        self.assertEqual(self._ukf._Xsig_pred.dimx, 5)
        self.assertEqual(self._ukf._Xsig_pred.dimy, 15)

    def test_ProcessMeasurement_SetsXAndP_IfFirstMeasurementIsLASER(self):
        meas_package = MeasurementPackage(0, 
                                          MeasurementPackage.SensorType.LASER,
                                          Matrix([[1], [1]]))

        self._ukf.process_measurement(meas_package)

        initial_x = Matrix([[1], [1], [0], [0], [0]])

        self.assertEqual(self._ukf._x.value, initial_x.value)

        initial_P = Matrix([[self._ukf._std_laspx*self._ukf._std_laspx, 0, 0, 0, 0],
                            [0, self._ukf._std_laspy*self._ukf._std_laspy, 0, 0, 0],
                            [0, 0, 9, 0, 0],
                            [0, 0, 0, 2.25, 0],
                            [0, 0, 0, 0, 0.0625]])

        self.assertEqual(self._ukf._P.value, initial_P.value)

    def test_ProcessMeasurement_SetsXAndP_IfFirstMeasurementIsRADAR(self):
        ro = 1.0
        theta = 2.0

        meas_package = MeasurementPackage(0,
                                          MeasurementPackage.SensorType.RADAR,
                                          Matrix([[ro], [theta], [0.5]]))

        self._ukf.process_measurement(meas_package)

        initial_x = Matrix([[ro*cos(theta)], [ro*sin(theta)], [0], [0], [0]])

        self.assertEqual(self._ukf._x.value, initial_x.value)

        std_rad = max(self._ukf._std_radr, self._ukf._std_radphi*ro)
        initial_P = Matrix([[std_rad*std_rad, 0, 0, 0, 0],
                            [0, std_rad*std_rad, 0, 0, 0],
                            [0, 0, 9, 0, 0],
                            [0, 0, 0, 2.25, 0],
                            [0, 0, 0, 0, 0.0625]])

        self.assertEqual(self._ukf._P.value, initial_P.value)

    def test_AugmentedSigmaPoints_CalculatesXsigAug_FromXAndP(self):
        n_aug = 7
        self._ukf._std_a = 0.2
        self._ukf._std_yawdd = 0.2

        self._ukf._x = Matrix([[5.7441], [1.3800], [2.2049], [0.5015], [0.3528]])

        self._ukf._P = Matrix([[0.0043, -0.0013, 0.0030, -0.0022, -0.0020],
                                [-0.0013, 0.0077, 0.0011, 0.0071, 0.0060],
                                [0.0030, 0.0011, 0.0054, 0.0007, 0.0008],
                                [-0.0022, 0.0071, 0.0007, 0.0098, 0.0100],
                                [-0.0020, 0.0060, 0.0008, 0.0100, 0.0123]])

        Xsig_aug = self._ukf.augmented_sigma_points()

        expected_Xsig_aug = Matrix([[5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441],
                                    [1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38],
                                    [2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049],
                                    [0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015],
                                    [0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528],
                                    [0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0],
                                    [0, 0, 0, 0, 0, 0, 0,  0.34641, 0, 0, 0, 0, 0, 0, -0.34641]])

        assert_array_almost_equal(Xsig_aug.value, expected_Xsig_aug.value, decimal=4)

    def test_PredictMeanAndCovariance_CalculatesXAndP_FromXsigPred(self):
        # create example matrix with predicted sigma points
        self._ukf._Xsig_pred = Matrix([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                       [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                       [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                       [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                       [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

        x, P = self._ukf.predict_mean_and_covariance()

        expected_x = Matrix([[5.93637], [1.49035], [2.20528], [0.536853], [0.353577]])
 
        assert_array_almost_equal(x.value, expected_x.value, decimal=5)
 
        expected_P = Matrix([[0.00543425,   -0.0024053,   0.00341576,  -0.00348196,  -0.00299378],
                             [-0.0024053,     0.010845,    0.0014923,   0.00980182,   0.00791091],
                             [0.00341576,    0.0014923,   0.00580129,  0.000778632,  0.000792973],
                             [-0.00348196,   0.00980182,  0.000778632,    0.0119238,    0.0112491],
                             [-0.00299378,   0.00791091,  0.000792973,    0.0112491,    0.0126972]])

        assert_array_almost_equal(P.value, expected_P.value)

    def test_Prediction_CallsAugmentedSigmaPointsThenUpdateXsigPredAndCallsPredictMeanAndCovariance_ForGivenDeltaT(self):
        self._ukf._std_a = 0.2
        self._ukf._std_yawdd = 0.2
        self._ukf._x = Matrix([[5.7441], [1.3800], [2.2049], [0.5015], [0.3528]])

        self._ukf._P = Matrix([[0.0043, -0.0013, 0.0030, -0.0022, -0.0020],
                                [-0.0013, 0.0077, 0.0011, 0.0071, 0.0060],
                                [0.0030, 0.0011, 0.0054, 0.0007, 0.0008],
                                [-0.0022, 0.0071, 0.0007, 0.0098, 0.0100],
                                [-0.0020, 0.0060, 0.0008, 0.0100, 0.0123]])

        self._ukf.augmented_sigma_points = MagicMock(wraps=self._ukf.augmented_sigma_points)
        self._ukf.predict_mean_and_covariance = MagicMock(wraps=self._ukf.predict_mean_and_covariance)

        ukf_sequence = Mock()
        ukf_sequence.attach_mock(self._ukf.augmented_sigma_points, 'augmented_sigma_points')
        ukf_sequence.attach_mock(self._ukf.predict_mean_and_covariance, 'predict_mean_and_covariance')

        delta_t = 0.1 #time diff in sec
        self._ukf.prediction(delta_t)

        expected_Xsig_pred = Matrix([[5.93553,  6.06251,  5.92217,   5.9415,  5.92361,  5.93516,  5.93705,  5.93553,  5.80832,  5.94481,  5.92935,  5.94553,  5.93589,  5.93401,  5.93553],
                                     [1.48939,  1.44673,  1.66484,  1.49719,    1.508,  1.49001,  1.49022,  1.48939,   1.5308,  1.31287,  1.48182,  1.46967,  1.48876,  1.48855,  1.48939],
                                     [2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.23954,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,  2.17026,   2.2049],
                                     [0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372,  0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188,  0.53678, 0.535048],
                                     [0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528, 0.387441, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528, 0.318159]])

        assert_array_almost_equal(self._ukf._Xsig_pred.value, expected_Xsig_pred.value, decimal=4)
        assert ukf_sequence.mock_calls == [call.augmented_sigma_points(), call.predict_mean_and_covariance()]

    def test_PredictRadarMeasurement_CalculatesZsigZPredAndS_FromXsigPred(self):
        n_aug = 7
        n_z = 3

        #radar measurement noise standard deviation radius in m
        self._ukf._std_radr = 0.3

        #radar measurement noise standard deviation angle in rad
        self._ukf._std_radphi = 0.0175

        #radar measurement noise standard deviation radius change in m/s
        self._ukf._std_radrd = 0.1

        #create example matrix with predicted sigma points
        self._ukf._Xsig_pred = Matrix([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                       [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                       [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                       [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                       [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

        Zsig, z_pred, S = self._ukf.predict_radar_measurement()

        expected_z_pred = Matrix([[6.12155], [0.245993], [2.10313]])
        
        assert_array_almost_equal(z_pred.value, expected_z_pred.value, decimal=4)

        #create example matrix for predicted measurement covariance
        expected_S = Matrix([[0.0946171, -0.000139448,   0.00407016],
                             [-0.000139448,  0.000617548, -0.000770652],
                             [0.00407016, -0.000770652,    0.0180917]])

        assert_array_almost_equal(S.value, expected_S.value)

    def test_UpdateRadar_CallsPredictRadarMeasurementThenCalculatesXAndP_ForGivenMeasurement(self):
        #radar measurement noise standard deviation radius in m
        self._ukf._std_radr = 0.3

        #radar measurement noise standard deviation angle in rad
        self._ukf._std_radphi = 0.0175

        #radar measurement noise standard deviation radius change in m/s
        self._ukf._std_radrd = 0.1

        self._ukf._Xsig_pred = Matrix([[5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744],
                                       [1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486],
                                       [2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049],
                                       [0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048],
                                       [0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159]])

        #create example vector for predicted state mean
        self._ukf._x = Matrix([[5.93637], [1.49035], [2.20528], [0.536853], [0.353577]])

        #create example matrix for predicted state covariance
        self._ukf._P = Matrix([[0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378],
                               [-0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091],
                               [0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973],
                               [-0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491],
                               [-0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972]])

        # create example matrix with sigma points in measurement space
        Zsig = Matrix([[6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057],
                       [0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239],
                       [2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295]])

        # create example vector for mean predicted measurement
        z_pred = Matrix([[6.12155], [0.245993], [2.10313]])

        # create example matrix for predicted measurement covariance
        S = Matrix([[0.0946171, -0.000139448,   0.00407016],
                    [-0.000139448,  0.000617548, -0.000770652],
                    [0.00407016, -0.000770652,    0.0180917]])

        self._ukf.predict_radar_measurement = MagicMock(return_value=(Zsig, z_pred, S))

        meas_package = MeasurementPackage(0, 
                                          MeasurementPackage.SensorType.RADAR,
                                          Matrix([[5.9214],   # rho in m
                                                  [0.2187],   # phi in rad
                                                  [2.0062]]))   # rho_dot in m/s

        self._ukf.update_radar(meas_package)

        # expected result x:
        expected_x = Matrix([[5.92276], [1.41823], [2.15593], [0.489274], [0.321338]])

        assert_array_almost_equal(self._ukf._x.value, expected_x.value, decimal=4)

        expected_P = Matrix([[0.00361579, -0.000357881,   0.00208316, -0.000937196,  -0.00071727],
                             [-0.000357881,   0.00539867,   0.00156846,   0.00455342,   0.00358885],
                             [0.00208316,   0.00156846,   0.00410651,   0.00160333,   0.00171811],
                             [-0.000937196,   0.00455342,   0.00160333,   0.00652634,   0.00669436],
                             [-0.00071719,   0.00358884,   0.00171811,   0.00669426,   0.00881797]])

        assert_array_almost_equal(self._ukf._P.value, expected_P.value, decimal=5)
        self.assertTrue(self._ukf.predict_radar_measurement.called)

    def test_ProcessMeasurement_CallsPredictThenUpdateRadar_ForSubsequentRADARMeasurements(self):
        first_measurement = MeasurementPackage(1477010443050000,
                                               MeasurementPackage.SensorType.RADAR,
                                               Matrix([[0.898658], 
                                                       [0.617674], 
                                                       [1.7986]]))

        second_measurement = MeasurementPackage(1477010443150000,
                                                MeasurementPackage.SensorType.RADAR,
                                                Matrix([[0.910574], 
                                                        [0.610537], 
                                                        [1.46233]]))

        self._ukf.prediction = MagicMock(wraps=self._ukf.prediction)
        self._ukf.update_radar = MagicMock(wraps=self._ukf.update_radar)

        ukf_sequence = Mock()
        ukf_sequence.attach_mock(self._ukf.prediction, 'prediction')
        ukf_sequence.attach_mock(self._ukf.update_radar, 'update_radar')

        self._ukf.process_measurement(first_measurement)
        self._ukf.process_measurement(second_measurement)

        assert ukf_sequence.mock_calls == [call.prediction(0.1),
                                           call.update_radar(second_measurement)]

    def test_ProcessMeasurement_CallsPredictThenUpdateLidar_ForSubsequentLASERMeasurements(self):
        first_measurement = MeasurementPackage(1477010443000000,
                                               MeasurementPackage.SensorType.LASER,
                                               Matrix([[0.463227], [0.607415]]))

        second_measurement = MeasurementPackage(1477010443100000,
                                                MeasurementPackage.SensorType.LASER,
                                                Matrix([[0.968521], [0.40545]]))

        self._ukf.prediction = MagicMock(wraps=self._ukf.prediction)
        self._ukf.update_lidar = MagicMock(wraps=self._ukf.update_lidar)

        ukf_sequence = Mock()
        ukf_sequence.attach_mock(self._ukf.prediction, 'prediction')
        ukf_sequence.attach_mock(self._ukf.update_lidar, 'update_lidar')

        self._ukf.process_measurement(first_measurement)
        self._ukf.process_measurement(second_measurement)

        assert ukf_sequence.mock_calls == [call.prediction(0.1),
                                           call.update_lidar(second_measurement)]

    def test_UKF_PassesProjectRubric_ForDataSet1(self):
        in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt"
        
        in_file = open(in_file_name_, newline='')
        data_reader = csv.reader(in_file, delimiter='\t', quotechar='|')

        self.assertFalse(in_file.closed)

        tools = Tools()

        # used to compute the RMSE later
        estimations = []
        ground_truth =[]

        # prep the measurement packages (each line represents a measurement at a
        # timestamp)
        for row in data_reader:
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
            ground_truth.append(gt_values)

            self._ukf.process_measurement(meas_package)
            p_x = self._ukf._x.value[0][0]
            p_y = self._ukf._x.value[1][0]
            v   = self._ukf._x.value[2][0]
            yaw = self._ukf._x.value[3][0]

            v1 = cos(yaw)*v
            v2 = sin(yaw)*v

            estimate = Matrix([[p_x], [p_y], [v1], [v2]])

            estimations.append(estimate)

        # compute the accuracy (RMSE)
        expected_rmse = Matrix([[0.09], [0.10], [0.40], [0.30]])
        rmse = tools.calculate_rmse(estimations, ground_truth)
        assert_array_less(rmse.value, expected_rmse.value)
        
        in_file.close()
