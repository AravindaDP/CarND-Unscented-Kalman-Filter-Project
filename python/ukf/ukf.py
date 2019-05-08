from ukf.matrix import Matrix
from ukf.measurement_package import MeasurementPackage
from math import sqrt, sin, cos, pi, atan2, fmod

class UKF:
    def __init__(self):
        """Initializes Unscented Kalman filter"""
        # initially set to false, set to true in first call of ProcessMeasurement
        self._is_initialized = False

        # predicted sigma points matrix
        self._Xsig_pred = Matrix([[]])

        # time when the state is true, in us
        self._time_us = 0

        # Weights of sigma points
        self._weights = Matrix([[]])

        # State dimension
        self._n_x = 5

        # Augmented state dimension
        self._n_aug = 7

        # Sigma point spreading parameter
        self._lambda = 3 - self._n_aug

        self._lambda_sqrt = sqrt(self._lambda+self._n_aug)

        #if this is false, laser measurements will be ignored (except during init)
        self._use_laser = True

        #if this is false, radar measurements will be ignored (except during init)
        self._use_radar = True

        # initial state vector
        # state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        self._x = Matrix([[]])
        self._x.zero(5, 1)

        # initial covariance matrix
        self._P = Matrix([[]])
        self._P.zero(5, 5)

        # Process noise standard deviation longitudinal acceleration in m/s^2
        self._std_a = 3

        # Process noise standard deviation yaw acceleration in rad/s^2
        self._std_yawdd = 1
  
        """
        DO NOT MODIFY measurement noise values below.
        These are provided by the sensor manufacturer.
        """

        # Laser measurement noise standard deviation position1 in m
        self._std_laspx = 0.15

        # Laser measurement noise standard deviation position2 in m
        self._std_laspy = 0.15

        # Radar measurement noise standard deviation radius in m
        self._std_radr = 0.3

        # Radar measurement noise standard deviation angle in rad
        self._std_radphi = 0.03

        # Radar measurement noise standard deviation radius change in m/s
        self._std_radrd = 0.3
  
        """
        End DO NOT MODIFY section for measurement noise values 
        """
  
        #set vector for weights
        self._weights.zero(2*self._n_aug+1, 1)

        # set weights
        weight_0 = self._lambda/(self._lambda+self._n_aug)
        weight = 0.5/(self._n_aug+self._lambda)
        self._weights.value[0] = [weight_0]

        for i in range(1, 2*self._n_aug+1):
            self._weights.value[i] = [weight]

        #create matrix with predicted sigma points as columns
        self._Xsig_pred.zero(self._n_x, 2 * self._n_aug + 1)

        # Lidar measurement covariance
        self._R = Matrix([[self._std_laspx*self._std_laspx, 0],
                          [0, self._std_laspy*self._std_laspy]])

        # Lidar measurement matrix
        self._H = Matrix([[1, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0]])

        self._Ht = self._H.transpose()
        
        self._lidar_nis = 0
        self._radar_nis = 0

    def process_measurement(self, meas_package):
        """ProcessMeasurement

        Args:
            meas_package (:obj:`MeasurementPackage`): The measurement at k+1
        """
        """
        Initialization structure similar to EKF project
        """

        if not self._is_initialized:
            #Initialize x_, P_, previous_time, anything else needed.

            if meas_package._sensor_type == MeasurementPackage.SensorType.LASER:
                #Initialize here
                self._x.zero(self._n_x,1)
                self._x.value[0:2] = meas_package._raw_measurements.value

                self._P = Matrix([[self._std_laspx*self._std_laspx, 0, 0, 0, 0],
                                  [0, self._std_laspy*self._std_laspy, 0, 0, 0],
                                  [0, 0, 9, 0, 0],  #Assumming 95% of time speed is +/- 6m/s
                                  [0, 0, 0, 2.25, 0], #Assuming 95% of the time angle is +/- 3rad
                                  [0, 0, 0, 0, 0.0625]]) #Assuming 95% of the time angular speed +/- 0.5 rad/s

            elif meas_package._sensor_type == MeasurementPackage.SensorType.RADAR:
                #Initialize here
                #Convert radar from polar to cartesian coordinates
                #         and initialize state.
                ro = meas_package._raw_measurements.value[0][0]
                theta = meas_package._raw_measurements.value[1][0]
                self._x.zero(self._n_x,1)
                self._x.value[0:2] = [[ro*cos(theta)], [ro*sin(theta)]]

                #max(standard deviation radius, standard deviation angle*radar distance)
                #a crude estimation of maximum standard deviation in cartesian coordinates
                std_rad = max(self._std_radr, self._std_radphi*ro)
                self._P = Matrix([[std_rad*std_rad, 0, 0, 0, 0],
                                  [0, std_rad*std_rad, 0, 0, 0],
                                  [0, 0, 9, 0, 0],
                                  [0, 0, 0, 2.25, 0],
                                  [0, 0, 0, 0, 0.0625]])

            #Initialize anything else here
            self._time_us = meas_package._timestamp
            self._is_initialized = True
            return

        """
        Control structure similar to EKF project
        """

        delta_t = (meas_package._timestamp - self._time_us)/1000000.0
        self.prediction(delta_t)

        if meas_package._sensor_type == MeasurementPackage.SensorType.LASER:
            self.update_lidar(meas_package)

        elif meas_package._sensor_type == MeasurementPackage.SensorType.RADAR:
            self.update_radar(meas_package)

        self._time_us = meas_package._timestamp

    def prediction(self, delta_t):
        """Predicts sigma points, the state, and the state covariance matrix
        
        Args:
            delta_t (float): Time between k and k+1 in s
        """
        """
        * Estimate the object's location.
        * Modify the state vector, x_. Predict sigma points, the state,
          and the state covariance matrix.
        """
        """
        Create Augmented Sigma Points
        """

        #create sigma point matrix
        Xsig_aug = self.augmented_sigma_points()

        """
        Predict Sigma Points
        """

        #Lesson 7, section 21: Sigma Point Prediction Assignment 2

        # may be it's very little gain. but to avoid couple of arithmatic operations
        delta_t_2 = delta_t*delta_t

        # predict sigma points
        for i in range(2*self._n_aug+1):
            #extract values for better readability
            p_x = Xsig_aug.value[0][i]
            p_y = Xsig_aug.value[1][i]
            v =  Xsig_aug.value[2][i]
            yaw = Xsig_aug.value[3][i]
            yawd = Xsig_aug.value[4][i]
            nu_a = Xsig_aug.value[5][i]
            nu_yawdd = Xsig_aug.value[6][i]

            # avoid division by zero
            if(abs(yawd) > 0.001):
                px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw))
                py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) )
            else:
                px_p = p_x + v*delta_t*cos(yaw)
                py_p = p_y + v*delta_t*sin(yaw)

            v_p = v
            yaw_p = yaw + yawd*delta_t
            yawd_p = yawd

            # add noise
            px_p = px_p + 0.5*nu_a*delta_t_2 * cos(yaw)
            py_p = py_p + 0.5*nu_a*delta_t_2 * sin(yaw)
            v_p = v_p + nu_a*delta_t

            yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_2
            yawd_p = yawd_p + nu_yawdd*delta_t

            # write predicted sigma point into right column
            self._Xsig_pred.value[0][i] = px_p
            self._Xsig_pred.value[1][i] = py_p
            self._Xsig_pred.value[2][i] = v_p
            self._Xsig_pred.value[3][i] = yaw_p
            self._Xsig_pred.value[4][i] = yawd_p

        """
        Predict mean and covariance
        """

        self._x, self._P = self.predict_mean_and_covariance()

    def update_lidar(self, meas_package):
        """Updates the state and the state covariance matrix using a laser measurement
        
        Args:
            meas_package (:obj:`MeasurementPackage`): The measurement at k+1
        """
        """
        Use lidar data to update the belief
        about the object's position. Modify the state vector, self._x, and
        covariance, self._P.
        You can also calculate the lidar NIS, if desired.
        """
        """
        Update Lidar Measurement
        """

        # The mapping from state space to Lidar is linear. Fill this out with
        # appropriate update steps

        z_pred = self._H * self._x
        y = meas_package._raw_measurements - z_pred
        #Ht = H.transpose()
        S = self._H * self._P * self._Ht + self._R
        Si = S.inverse()
        PHt = self._P * self._Ht
        K = PHt * Si

        # new estimate
        self._x = self._x + (K * y)
        x_size = self._x.dimx
        I = Matrix([[]])
        I.identity(x_size)
        self._P = (I - K * self._H) * self._P

        nis = y.transpose()*Si*y
        self._lidar_nis = nis.value[0][0]

    def update_radar(self, meas_package):
        """Updates the state and the state covariance matrix using a radar measurement
        
        Args:
            meas_package (:obj:`MeasurementPackage`): The measurement at k+1
        """
        """
        Use radar data to update the belief
        about the object's position. Modify the state vector, self._x, and
        covariance, self._P.
        You can also calculate the radar NIS, if desired.
        """
        """
        Predict Radar Sigma Points
        """

        #set measurement dimension, radar can measure r, phi, and r_dot
        n_z = 3

        Zsig, z_pred, S = self.predict_radar_measurement()

        """
        Update Radar
        """

        #Lesson 7, section 30: UKF Update Assignment 2

        # create matrix for cross correlation Tc
        Tc = Matrix([[]])

        # calculate cross correlation matrix
        Tc.zero(self._n_x, n_z)
        for i in range(2 * self._n_aug + 1) :  #2n+1 simga points
            # residual
            z_diff = Matrix([[]])
            z_diff.zero(n_z,1)
            for row in range(n_z):
                z_diff.value[row][0] = Zsig.value[row][i]
            z_diff = z_diff - z_pred
            # angle normalization
            z_diff.value[1][0] = fmod(z_diff.value[1][0], pi)

            # state difference
            x_diff = Matrix([[]])
            x_diff.zero(self._n_x,1)
            for row in range(self._n_x):
                x_diff.value[row][0] = self._Xsig_pred.value[row][i]
            x_diff = x_diff - self._x
            # angle normalization
            x_diff.value[3][0] = fmod(x_diff.value[3][0], pi)

            x_diff_z_diff_t = x_diff * z_diff.transpose()
            x_diff_z_diff_t.value = [[xzv*self._weights.value[i][0] for xzv in row]
                                     for row in x_diff_z_diff_t.value]
            Tc = Tc + x_diff_z_diff_t

        # Kalman gain K;
        K = Tc * S.inverse()

        # residual
        z_diff = meas_package._raw_measurements - z_pred

        # angle normalization
        z_diff.value[1][0] = fmod(z_diff.value[1][0], pi)

        #update state mean and covariance matrix
        self._x = self._x + K * z_diff
        self._P = self._P - K*S*K.transpose()

        nis = z_diff.transpose()*S.inverse()*z_diff
        self._radar_nis = nis.value[0][0]

    """
    Student assignment functions
    """
    def augmented_sigma_points(self):
        #Lesson 7, section 18: Augmentation Assignment 2

        #create augmented mean vector
        x_aug = Matrix([[]])
        x_aug.zero(self._n_aug, 1)

        # create augmented state covariance
        P_aug = Matrix([[]])
        P_aug.zero(self._n_aug, self._n_aug)

        # create sigma point matrix
        Xsig_aug = Matrix([[]])
        Xsig_aug.zero(self._n_aug, 2 * self._n_aug + 1)

        # create augmented mean state, remember mean of noise is zero
        x_aug.value = self._x.value + [[0], [0]]

        # create augmented covariance matrix
        P_aug.value = ([row + [0, 0] for row in self._P.value]
                       + [[0, 0, 0, 0, 0, self._std_a*self._std_a, 0],
                          [0, 0 , 0, 0, 0, 0, self._std_yawdd*self._std_yawdd]])

        # create square root matrix
        L = P_aug.Cholesky().transpose()

        # create augmented sigma points
        for row in range(len(x_aug.value)):
            Xsig_aug.value[row][0] = x_aug.value[row][0]
        for i in range(self._n_aug):
            for row in range(len(x_aug.value)):
                Xsig_aug.value[row][i+1] = (x_aug.value[row][0]
                                            + self._lambda_sqrt*L.value[row][i]) #+ sqrt(self._lambda+self._n_aug)*L.value[row][i])
                Xsig_aug.value[row][i+1+self._n_aug] = (x_aug.value[row][0]
                                                        - self._lambda_sqrt*L.value[row][i]) #- sqrt(self._lambda+self._n_aug)*L.value[row][i])

        return Xsig_aug

    def predict_mean_and_covariance(self):
        #Lesson 7, section 24: Predicted Mean and Covariance Assignment 2

        # create vector for predicted state
        x = Matrix([[]])

        # create covariance matrix for prediction
        P = Matrix([[]])

        # predicted state mean
        x.zero(self._n_x,1)
        for i in range(2*self._n_aug+1):  # iterate over sigma points
            for row in range(x.dimx):
                x.value[row] = [x.value[row][0]
                                + self._weights.value[i][0]*self._Xsig_pred.value[row][i]]

        # predicted state covariance matrix
        P.zero(self._n_x, self._n_x)
        for i in range(2*self._n_aug+1):  # iterate over sigma points
            # state difference
            x_diff = Matrix([[]])
            x_diff.zero(self._n_x,1)
            for row in range(self._n_x):
                x_diff.value[row][0] = self._Xsig_pred.value[row][i]
            x_diff = x_diff - x
            # angle normalization
            x_diff.value[3][0] = fmod(x_diff.value[3][0], pi)

            x_diff2 = x_diff * x_diff.transpose()
            x_diff2.value = [[xv*self._weights.value[i][0] for xv in row] 
                             for row in x_diff2.value]
            P = P + x_diff2

        return (x, P)

    def predict_radar_measurement(self):
        # Lesson 7, section 27: Predict Radar Measurement Assignment 2

        # set measurement dimension, radar can measure r, phi, and r_dot
        n_z = 3

        # create matrix for sigma points in measurement space
        Zsig = Matrix([[]])
        Zsig.zero(n_z, 2 * self._n_aug + 1)

        # mean predicted measurement
        z_pred = Matrix([[]])

        # measurement covariance matrix S
        S = Matrix([[]])
        S.zero(n_z,n_z)

        # transform sigma points into measurement space
        for i in range(2 * self._n_aug + 1):  # 2n+1 simga points
            # extract values for better readability
            p_x = self._Xsig_pred.value[0][i]
            p_y = self._Xsig_pred.value[1][i]
            v  = self._Xsig_pred.value[2][i]
            yaw = self._Xsig_pred.value[3][i]

            v1 = cos(yaw)*v
            v2 = sin(yaw)*v

            # measurement model
            Zsig.value[0][i] = sqrt(p_x*p_x + p_y*p_y)                       # r
            Zsig.value[1][i] = atan2(p_y,p_x)                                # phi
            Zsig.value[2][i] = (p_x*v1 + p_y*v2) / Zsig.value[0][i] #sqrt(p_x*p_x + p_y*p_y)   # r_dot

        # mean predicted measurement
        z_pred.zero(n_z,1)
        for i in range(2 * self._n_aug + 1):
            for row in range(n_z):
                z_pred.value[row] = [z_pred.value[row][0]
                                     + self._weights.value[i][0] * Zsig.value[row][i]]

        # innovation covariance matrix S
        S.zero(n_z,n_z)
        for i in range(2 * self._n_aug + 1):  # 2n+1 simga points
            # residual
            z_diff = Matrix([[]])
            z_diff.zero(n_z,1)
            for row in range(n_z):
                z_diff.value[row][0] = Zsig.value[row][i]
            z_diff = z_diff - z_pred

            # angle normalization
            z_diff.value[1][0] = fmod(z_diff.value[1][0], pi)

            z_diff2 = z_diff * z_diff.transpose()
            z_diff2.value = [[zv*self._weights.value[i][0] for zv in row]
                             for row in z_diff2.value]
            S = S + z_diff2

        # add measurement noise covariance matrix
        R = Matrix([[self._std_radr*self._std_radr, 0, 0],
                    [0, self._std_radphi*self._std_radphi, 0],
                    [0, 0,self._std_radrd*self._std_radrd]])
        S = S + R

        return (Zsig, z_pred, S)