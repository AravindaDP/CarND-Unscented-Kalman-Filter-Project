#include "../src/ukf.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "../src/tools.h"
#include <vector>
#include <fstream>
#include <sstream>
#include "mock_ukf.h"

using Eigen::Map;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::ifstream;
using ::testing::_;
using ::testing::SetArgPointee;
using ::testing::DoAll;
using ::testing::InSequence;

MATCHER_P(IsLt, n, "") { return ((arg-n).array() < 0).any(); }

class UKFTest : public ::testing::Test {
 protected:
  MockUKF ukf_;

  void SetUp() override {
    std::cout.setstate(std::ios_base::failbit);
  }

  void TearDown() override {
    std::cout.clear();  
  }
};

//initialize parameters and matrices
TEST_F(UKFTest, Constructor_SetsParametersAndMatrices) {
  ASSERT_FALSE(ukf_.is_initialized_);
  ASSERT_EQ(ukf_.n_x_, 5);
  ASSERT_EQ(ukf_.n_aug_, 7);
  ASSERT_EQ(ukf_.lambda_, 3-7);
  ASSERT_EQ(ukf_.weights_.size(), 15);
  ASSERT_EQ(ukf_.Xsig_pred_.rows(), 5);
  ASSERT_EQ(ukf_.Xsig_pred_.cols(), 15);
}

TEST_F(UKFTest, ProcessMeasurement_SetsXAndP_IfFirstMeasurementIsLASER) {
  MeasurementPackage meas_package = {MeasurementPackage::LASER, 0,
                                     Map<VectorXd>(vector<double>({ 1, 1 }).data(), 2)};

  ukf_.ProcessMeasurement(meas_package);

  VectorXd initial_x = VectorXd(5);
  initial_x << 1, 1, 0, 0, 0;

  ASSERT_TRUE(ukf_.x_.isApprox(initial_x));

  MatrixXd initial_P = MatrixXd(5, 5);
  initial_P << ukf_.std_laspx_*ukf_.std_laspx_, 0, 0, 0, 0,
               0, ukf_.std_laspy_*ukf_.std_laspy_, 0, 0, 0,
               0, 0, 9, 0, 0,
               0, 0, 0, 2.25, 0,
               0, 0, 0, 0, 0.0625;

  ASSERT_TRUE(ukf_.P_.isApprox(initial_P));
}

TEST_F(UKFTest, ProcessMeasurement_SetsXAndP_IfFirstMeasurementIsRADAR) {
  float ro = 1;
  float theta = 2;

  MeasurementPackage meas_package = {
      MeasurementPackage::RADAR, 0,
      Map<VectorXd>(vector<double>({ro, theta, 0.5}).data(), 3)};

  ukf_.ProcessMeasurement(meas_package);

  VectorXd initial_x = VectorXd(5);
  initial_x <<  ro*cos(theta), ro*sin(theta), 0, 0, 0;

  ASSERT_TRUE(ukf_.x_.isApprox(initial_x));

  MatrixXd initial_P = MatrixXd(5, 5);
  double std_rad = std::max(ukf_.std_radr_, ukf_.std_radphi_*ro);
  initial_P << std_rad*std_rad, 0, 0, 0, 0,
               0, std_rad*std_rad, 0, 0, 0,
               0, 0, 9, 0, 0,
               0, 0, 0, 2.25, 0,
               0, 0, 0, 0, 0.0625;

  ASSERT_TRUE(ukf_.P_.isApprox(initial_P));
}

TEST_F(UKFTest, AugmentedSigmaPoints_CalculatesXsigAug_FromXAndP) {
  int n_aug = 7;

  ukf_.std_a_ = 0.2;
  ukf_.std_yawdd_ = 0.2;
  ukf_.x_ << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;
  
  ukf_.P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
                -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
                 0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
                -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
                -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  ukf_.AugmentedSigmaPoints(&Xsig_aug);

  MatrixXd expected_Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  expected_Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  ASSERT_TRUE(Xsig_aug.isApprox(expected_Xsig_aug, 0.0001));
}

TEST_F(UKFTest, Prediction_CallsAugmentedSigmaPointsThenUpdateXsigPredAndCallsPredictMeanAndCovariance_ForGivenDeltaT) {
  ukf_.std_a_ = 0.2;
  ukf_.std_yawdd_ = 0.2;
  ukf_.x_ << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;
  
  ukf_.P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
                -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
                 0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
                -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
                -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  {
    InSequence ukf;

    EXPECT_CALL(ukf_, AugmentedSigmaPoints(_));
    EXPECT_CALL(ukf_, PredictMeanAndCovariance(_, _));
  }

  double delta_t = 0.1; //time diff in sec
  ukf_.Prediction(delta_t);

  MatrixXd expected_Xsig_pred = MatrixXd(5, 15);
  expected_Xsig_pred <<
      5.93553,  6.06251,  5.92217,   5.9415,  5.92361,  5.93516,  5.93705,  5.93553,  5.80832,  5.94481,  5.92935,  5.94553,  5.93589,  5.93401,  5.93553,
      1.48939,  1.44673,  1.66484,  1.49719,    1.508,  1.49001,  1.49022,  1.48939,   1.5308,  1.31287,  1.48182,  1.46967,  1.48876,  1.48855,  1.48939,
       2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.23954,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,  2.17026,   2.2049,
      0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372,  0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188,  0.53678, 0.535048,
       0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528, 0.387441, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528, 0.318159;

  ASSERT_TRUE(ukf_.Xsig_pred_.isApprox(expected_Xsig_pred, 0.0001));
}

TEST_F(UKFTest, PredictMeanAndCovariance_CalculatesXAndP_FromXsigPred) {
  // create example matrix with predicted sigma points
  ukf_.Xsig_pred_ <<
     5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
       1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
      2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
     0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
      0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create vector for predicted state
  VectorXd x = VectorXd(5);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(5, 5);
  
  ukf_.PredictMeanAndCovariance(&x, &P);

  VectorXd expected_x = VectorXd(5);
  expected_x << 5.93637, 1.49035, 2.20528, 0.536853, 0.353577;

  ASSERT_TRUE(x.isApprox(expected_x, 0.00001));

  MatrixXd expected_P = MatrixXd(5, 5);
  expected_P <<
    0.00543425,   -0.0024053,   0.00341576,  -0.00348196,  -0.00299378,
    -0.0024053,     0.010845,    0.0014923,   0.00980182,   0.00791091,
    0.00341576,    0.0014923,   0.00580129,  0.000778632,  0.000792973,
   -0.00348196,   0.00980182,  0.000778632,    0.0119238,    0.0112491,
   -0.00299378,   0.00791091,  0.000792973,    0.0112491,    0.0126972;

  ASSERT_TRUE(P.isApprox(expected_P, 0.000002));
}

TEST_F(UKFTest, PredictRadarMeasurement_CalculatesZsigZPredAndS_FromXsigPred) {
  int n_aug = 7;
  int n_z = 3;

  //radar measurement noise standard deviation radius in m
  ukf_.std_radr_ = 0.3;

  //radar measurement noise standard deviation angle in rad
  ukf_.std_radphi_ = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  ukf_.std_radrd_ = 0.1;

  //create example matrix with predicted sigma points
  ukf_.Xsig_pred_ <<
     5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
       1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
      2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
     0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
      0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);

  ukf_.PredictRadarMeasurement(&Zsig, &z_pred, &S);

  VectorXd expected_z_pred = VectorXd(n_z);
  expected_z_pred << 6.12155, 0.245993, 2.10313;

  ASSERT_TRUE(z_pred.isApprox(expected_z_pred, 0.00005));

  //create example matrix for predicted measurement covariance
  MatrixXd expected_S = MatrixXd(n_z, n_z);
  expected_S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  ASSERT_TRUE(S.isApprox(expected_S, 0.0000005));
}

TEST_F(UKFTest, UpdateRadar_CallsPredictRadarMeasurementThenCalculatesXAndP_ForGivenMeasurement) {
  //radar measurement noise standard deviation radius in m
  ukf_.std_radr_ = 0.3;

  //radar measurement noise standard deviation angle in rad
  ukf_.std_radphi_ = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  ukf_.std_radrd_ = 0.1;

  ukf_.Xsig_pred_ <<
     5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
       1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
      2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
     0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
      0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create example vector for predicted state mean
  ukf_.x_ << 5.93637, 1.49035, 2.20528, 0.536853, 0.353577;

  //create example matrix for predicted state covariance
  ukf_.P_ <<
    0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
    -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
    0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
   -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
   -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

   // create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(3,15);
  Zsig <<
    6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
   0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
    2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

  // create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  z_pred << 6.12155, 0.245993, 2.10313;

  // create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(3,3);
  S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  EXPECT_CALL(ukf_, PredictRadarMeasurement(_, _, _))
      .WillOnce(DoAll(SetArgPointee<0>(Zsig),SetArgPointee<1>(z_pred),SetArgPointee<2>(S)));

  MeasurementPackage meas_package;
  meas_package.sensor_type_ = MeasurementPackage::RADAR;
  meas_package.raw_measurements_ = VectorXd(3);
  meas_package.raw_measurements_ << 
     5.9214,   // rho in m
     0.2187,   // phi in rad
     2.0062;   // rho_dot in m/s

  ukf_.UpdateRadar(meas_package);

  // expected result x:
  VectorXd expected_x = VectorXd(5);
  expected_x << 5.92276, 1.41823, 2.15593, 0.489274, 0.321338;

  ASSERT_TRUE(ukf_.x_.isApprox(expected_x,0.00005));
    
  // expected result P:
  MatrixXd expected_P = MatrixXd(5,5);
  expected_P <<   0.00361579, -0.000357881,   0.00208316, -0.000937196,  -0.00071727,
                -0.000357881,   0.00539867,   0.00156846,   0.00455342,   0.00358885,
                  0.00208316,   0.00156846,   0.00410651,   0.00160333,   0.00171811,
                -0.000937196,   0.00455342,   0.00160333,   0.00652634,   0.00669436,
                 -0.00071719,   0.00358884,   0.00171811,   0.00669426,   0.00881797;

  ASSERT_TRUE(ukf_.P_.isApprox(expected_P,0.000001));
}

TEST_F(UKFTest, ProcessMeasurement_CallsPredictThenUpdateRadar_ForSubsequentRADARMeasurements) {
  MeasurementPackage first_measurement = {MeasurementPackage::RADAR, 1477010443050000,
                                          Map<VectorXd>(vector<double>({0.898658, 0.617674, 1.7986}).data(), 3)};

  MeasurementPackage second_measurement = {MeasurementPackage::RADAR, 1477010443150000,
                                           Map<VectorXd>(vector<double>({0.910574, 0.610537, 1.46233}).data(), 3)};

  {
    InSequence ukf;

    EXPECT_CALL(ukf_, Prediction(0.1));
    EXPECT_CALL(ukf_, UpdateRadar(second_measurement));
  }

  ukf_.ProcessMeasurement(first_measurement);
  ukf_.ProcessMeasurement(second_measurement);
}

TEST_F(UKFTest, ProcessMeasurement_CallsPredictThenUpdateLidar_ForSubsequentLASERMeasurements) {
  MeasurementPackage first_measurement = {MeasurementPackage::LASER, 1477010443000000,
                                          Map<VectorXd>(vector<double>({0.463227, 0.607415}).data(), 2)};

  MeasurementPackage second_measurement = {MeasurementPackage::LASER, 1477010443100000,
                                           Map<VectorXd>(vector<double>({0.968521, 0.40545}).data(), 2)};

  {
    InSequence ukf;

    EXPECT_CALL(ukf_, Prediction(0.1));
    EXPECT_CALL(ukf_, UpdateLidar(second_measurement));
  }

  ukf_.ProcessMeasurement(first_measurement);
  ukf_.ProcessMeasurement(second_measurement);
}


TEST_F(UKFTest, UKF_PassesProjectRubric_ForDataSet1) {
  std::string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  std::ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  ASSERT_TRUE(in_file_.is_open());

  std::string line;

  Tools tools;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    std::string sensor_type;
    MeasurementPackage meas_package;
    VectorXd gt_values(4);
    std::istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
    }
    else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    gt_values << x_gt, y_gt, vx_gt, vy_gt;
    ground_truth.push_back(gt_values);

    ukf_.ProcessMeasurement(meas_package);
    
    // Push the current estimated x,y positon from the Kalman filter's 
    //   state vector

    VectorXd estimate(4);

    double p_x = ukf_.x_(0);
    double p_y = ukf_.x_(1);
    double v   = ukf_.x_(2);
    double yaw = ukf_.x_(3);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;
        
    estimations.push_back(estimate);
  }

  // compute the accuracy (RMSE)
  VectorXd expected_rmse(4);
  expected_rmse << 0.09, 0.10, 0.40, 0.30;
  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
  ASSERT_THAT(tools.CalculateRMSE(estimations, ground_truth), IsLt(expected_rmse));

  // close files
  if (in_file_.is_open()) {
    in_file_.close();
  }
}
