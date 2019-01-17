from ukf.matrix import Matrix

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
        self._n_x = 0

        # Augmented state dimension
        self._n_aug = 0

        # Sigma point spreading parameter
        self._lambda = 0

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
        self._std_a = 30

        # Process noise standard deviation yaw acceleration in rad/s^2
        self._std_yawdd = 30
  
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
  
        """
        Todo:
            Complete the initialization. See ukf.h for other member properties.
        
        Hint: one or more values initialized above might be wildly off...
        """

    def process_measurement(self, meas_package):
        """ProcessMeasurement

        Args:
            meas_package (:obj:`MeasurementPackage`): The measurement at k+1
        """
        """
        Todo:
            Complete this function! Make sure you switch between lidar and radar
            measurements.
        """

    def prediction(self, delta_t):
        """Predicts sigma points, the state, and the state covariance matrix
        
        Args:
            delta_t (float): Time between k and k+1 in s
        """
        """
        Todo:
            * Complete this function! Estimate the object's location. 
            * Modify the state vector, x_. Predict sigma points, the state, 
              and the state covariance matrix.
        """

    def update_lidar(self, meas_package):
        """Updates the state and the state covariance matrix using a laser measurement
        
        Args:
            meas_package (:obj:`MeasurementPackage`): The measurement at k+1
        """
        """
        Todo:
            Complete this function! Use lidar data to update the belief 
            about the object's position. Modify the state vector, self._x, and 
            covariance, self._P.
            You can also calculate the lidar NIS, if desired.
        """

    def update_radar(self, meas_package):
        """Updates the state and the state covariance matrix using a radar measurement
        
        Args:
            meas_package (:obj:`MeasurementPackage`): The measurement at k+1
        """
        """
        Todo:
            Complete this function! Use radar data to update the belief 
            about the object's position. Modify the state vector, self._x, and 
            covariance, self._P.
            You can also calculate the radar NIS, if desired.
        """