#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.57;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.57;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  is_initialized_ = false;

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  lambda_sqrt_ = sqrt(lambda_+n_aug_);

  //set vector for weights
  weights_ = VectorXd(2*n_aug_+1);

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(n_aug_+lambda_);
  weights_(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    weights_(i) = weight;
  }
 
  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  Xsig_diff_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Lidar measurement covariance
	R_ = MatrixXd(2, 2);
	R_ << std_laspx_*std_laspx_, 0,
			  0, std_laspy_*std_laspy_;

  // Lidar measurement matrix
	H_ = MatrixXd(2, 5);
	H_ << 1, 0, 0, 0, 0,
			  0, 1, 0, 0, 0;
  Ht_ = H_.transpose();

  lidar_nis_ = 0;
  radar_nis_ = 0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {  
  /****************************************************************************
  *                Initialization structure similar to EKF project            *
  *****************************************************************************/

	if (!is_initialized_) {
		//Initialize x_, P_, previous_time, anything else needed.

		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			//Initialize here
			x_.fill(0.0);
			x_.head(2) = meas_package.raw_measurements_;

      P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
            0, std_laspy_*std_laspy_, 0, 0, 0,
            0, 0, 9, 0, 0,  //Assumming 95% of time speed is +/- 6m/s
            0, 0, 0, 2.25, 0, //Assuming 95% of the time angle is +/- 3rad
            0, 0, 0, 0, 0.0625; //Assuming 95% of the time angular speed +/- 0.5 rad/s
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			//Initialize here
      //Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      x_ << ro*cos(theta), ro*sin(theta), 0, 0, 0;

      //max(standard deviation radius, standard deviation angle*radar distance)
      //a crude estimation of maximum standard deviation in cartesian coordinates
      double std_rad = std::max(std_radr_, std_radphi_*ro)*2;
      P_ << std_rad*std_rad, 0, 0, 0, 0,
            0, std_rad*std_rad, 0, 0, 0,
            0, 0, 9, 0, 0, //Assumming 95% of time speed is +/- 6m/s
            0, 0, 0, 2.25, 0, //Assuming 95% of the time angle is +/- 3rad
            0, 0, 0, 0, 0.0625; //Assuming 95% of the time angular speed +/- 0.5 rad/s
		}

		//Initialize anything else here
		time_us_ = meas_package.timestamp_;
		is_initialized_ = true;
		return;
	}

  /****************************************************************************
  *                Control structure similar to EKF project                   *
  *****************************************************************************/

  double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
  time_us_ = meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  /**
   * Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  /****************************************************************************
  *                Create Augmented Sigma Points                              *
  *****************************************************************************/

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  AugmentedSigmaPoints(&Xsig_aug);

  /****************************************************************************
  *                Predict Sigma Points                                       *
  *****************************************************************************/

  //Lesson 7, section 21: Sigma Point Prediction Assignment 2

  // may be it's very little gain. but to avoid couple of arithmatic operations
  double delta_t_2 = delta_t*delta_t;

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t_2 * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t_2 * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_2;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /****************************************************************************
  *                Predict mean and covariance                                *
  *****************************************************************************/
 
  PredictMeanAndCovariance(&x_, &P_);
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  /****************************************************************************
  *                Update Lidar Measurement                                   *
  *****************************************************************************/

  // The mapping from state space to Lidar is linear. Fill this out with
  // appropriate update steps

  VectorXd z_pred = H_ * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  //MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht_ + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht_;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  lidar_nis_ = y.transpose()*Si*y;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  
  /****************************************************************************
  *                Predict Radar Sigma Points                                 *
  *****************************************************************************/

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  PredictRadarMeasurement(&Zsig, &z_pred, &S);

  /****************************************************************************
  *                Update Radar                                               *
  *****************************************************************************/

  //Lesson 7, section 30: UKF Update Assignment 2

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    z_diff(1) = fmod(z_diff(1), M_PI);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    x_diff(3) = fmod(x_diff(3), M_PI);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  // angle normalization
  z_diff(1) = fmod(z_diff(1), M_PI);

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  radar_nis_ = z_diff.transpose()*S.inverse()*z_diff;
}

/**
 * Programming assignment functions:
 */

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
  //Lesson 7, section 18: Augmentation Assignment 2

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create augmented mean state, remember mean of noise is zero
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_out->col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_out->col(i+1)       = x_aug + lambda_sqrt_ * L.col(i);//x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_out->col(i+1+n_aug_) = x_aug - lambda_sqrt_ * L.col(i);//x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {
  //Lesson 7, section 24: Predicted Mean and Covariance Assignment 2

  // predicted state mean
  x_out->fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    (*x_out) = (*x_out) + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_out->fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - (*x_out);
    // angle normalization
    x_diff(3) = fmod(x_diff(3), M_PI);

    *P_out = (*P_out) + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::PredictRadarMeasurement(MatrixXd* Zsig_out,
                                  VectorXd* z_out, MatrixXd* S_out) {
  // Lesson 7, section 27: Predict Radar Measurement Assignment 2

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig_out->row(0)(i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig_out->row(1)(i) = atan2(p_y,p_x);                                // phi
    Zsig_out->row(2)(i) = (p_x*v1 + p_y*v2) / Zsig_out->row(0)(i); //sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_out->fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    *z_out = (*z_out) + weights_(i) * Zsig_out->col(i);
  }

  // innovation covariance matrix S
  S_out->fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_out->col(i) - (*z_out);

    // angle normalization
    z_diff(1) = fmod(z_diff(1), M_PI);

    *S_out = (*S_out) + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  *S_out = (*S_out) + R;
}