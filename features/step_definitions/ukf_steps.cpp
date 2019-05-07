#define CUKE_OBJECT_PREFIX UKF

#include <gtest/gtest.h>
#include <cucumber-cpp/autodetect.hpp>
#include "../../src/measurement_package.h"
#include "../../test/mock_ukf.h"
#include <vector>
#include "helpers.h"
#include "../../src/tools.h"
#include <fstream>

using std::ifstream;

using cucumber::ScenarioScope;
using ::testing::_;
using ::testing::SetArgPointee;
using ::testing::DoAll;
using ::testing::SaveArgPointee;
using ::testing::SaveArg;

MATCHER_P(IsLt, n, "") { return ((arg-n).array() < 0).any(); }

ACTION_P2(Record, call_list, call) {
  call_list->push_back(call);
  return;
}

struct UKFCtx {
  Tools tools_;
  MockUKF ukf_;
  MeasurementPackage meas_package;
  MeasurementPackage meas_package_arg;
  double delta_t;
  std::map<std::string, MatrixXd> matrix_vars;
  std::map<std::string, VectorXd> vectors;
  std::vector<std::string> ukf_calls;
  std::vector<std::string> measurements;
  std::vector<VectorXd> estimations;
  std::vector<VectorXd> ground_truth;

  UKFCtx() {
    ON_CALL(ukf_, AugmentedSigmaPoints(_))
        .WillByDefault(DoAll(Record(&ukf_calls, "AugmentedSigmaPoints"), 
                             Invoke(&ukf_, &MockUKF::UKFAugmentedSigmaPoints), 
                             SaveArgPointee<0>(&(matrix_vars["Xsig_aug"]))));
    ON_CALL(ukf_, PredictMeanAndCovariance(_, _))
        .WillByDefault(DoAll(Record(&ukf_calls, "PredictMeanAndCovariance"), 
                             Invoke(&ukf_, &MockUKF::UKFPredictMeanAndCovariance)));
    ON_CALL(ukf_, PredictRadarMeasurement(_, _, _))
        .WillByDefault(DoAll(Record(&ukf_calls, "PredictRadarMeasurement"), 
                             Invoke(&ukf_, &MockUKF::UKFPredictRadarMeasurement)));

    int n_aug = 7;
    int n_z = 3;
    matrix_vars["Zsig"] = Eigen::MatrixXd(n_z, 2 * n_aug + 1);
    vectors["z_pred"] = Eigen::VectorXd(n_z);
    matrix_vars["S"] = Eigen::MatrixXd(n_z, n_z);
    matrix_vars["Xsig_aug"] = Eigen::MatrixXd(n_aug, 2 * n_aug + 1);
  }

  void UKFPredictRadarMeasurement(Eigen::MatrixXd* Zsig_out,
                                      Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out) {
    *Zsig_out = matrix_vars["Zsig"];
    *z_out = vectors["z_pred"];
    *S_out = matrix_vars["S"];
  }
};


GIVEN("^a new UKF$") {
  ScenarioScope<UKFCtx> context;
}


THEN("^n_x_ should be 5$") {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->ukf_.n_x_, 5);
}


THEN("^n_aug_ should be 7$") {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->ukf_.n_aug_, 7);
}


THEN("^lambda_ should be -4$") {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->ukf_.lambda_, -4);
}


THEN("^weights_ should be of size 15$") {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->ukf_.weights_.size(), 15);
}


THEN("^Xsig_pred_ should be of size 5 by 15$") {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->ukf_.Xsig_pred_.rows(), 5);
  ASSERT_EQ(context->ukf_.Xsig_pred_.cols(), 15);
}


WHEN("^I run UnscentedKF$") {
  ScenarioScope<UKFCtx> context;
  for (auto &line : context->measurements) // access by reference to avoid copying
  {
    MeasurementPackage meas_package;
    VectorXd gt_values(4);
    ParseMeasurement(line, meas_package, &gt_values);
    context->ground_truth.push_back(gt_values);

    context->ukf_.ProcessMeasurement(meas_package);
    
    // Push the current estimated x,y positon from the Kalman filter's 
    //   state vector

    VectorXd estimate(4);

    double p_x = context->ukf_.x_(0);
    double p_y = context->ukf_.x_(1);
    double v   = context->ukf_.x_(2);
    double yaw = context->ukf_.x_(3);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;
        
    context->estimations.push_back(estimate);
  }
}


THEN("^final rmse should be less than$", (const std::string expected_rmse_str)) {
  ScenarioScope<UKFCtx> context;
  // compute the accuracy (RMSE)
  VectorXd expected_rmse(4);
  expected_rmse = ParseVectorXd(expected_rmse_str);
  VectorXd rmse = context->tools_.CalculateRMSE(context->estimations, context->ground_truth);
  ASSERT_THAT(rmse, IsLt(expected_rmse));
}


THEN("^is_initialized_ should be false$") {
  ScenarioScope<UKFCtx> context;
  ASSERT_FALSE(context->ukf_.is_initialized_);
}


GIVEN("^Xsig_pred_ is$", (const std::string Xsig_pred_str)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.Xsig_pred_ = ParseMatrixXd(Xsig_pred_str);
}


WHEN("^I call PredictMeanAndCovariance$") {
  // create vector for predicted state
  VectorXd x = VectorXd(5);
  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(5, 5);
  
  ScenarioScope<UKFCtx> context;
  context->ukf_.PredictMeanAndCovariance(&x, &P);
  context->vectors["x"] = x;
  context->matrix_vars["P"] = P;
}


THEN("^vector (.+) with accuracy (.*) should be$", (const std::string variable,
                                                    const double accuracy,
                                                    const std::string value)) {
  VectorXd expected_val = ParseVectorXd(value);
  ScenarioScope<UKFCtx> context;
  EXPECT_TRUE(context->vectors[variable].isApprox(expected_val, accuracy));
}


THEN("^matrix (.+) with accuracy (.*) should be$", (const std::string variable,
                                                    const double accuracy,
                                                    const std::string value)) {
  MatrixXd expected_val = ParseMatrixXd(value);
  ScenarioScope<UKFCtx> context;
  EXPECT_TRUE(context->matrix_vars[variable].isApprox(expected_val, accuracy));
}


GIVEN("^std_radr_ is (.*)$", (const double std_radr)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.std_radr_ = std_radr;
}


GIVEN("^std_radphi_ is (.*)$", (const double std_radphi)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.std_radphi_ = std_radphi;
}


GIVEN("^std_radrd_ is (.*)$", (const double std_radrd)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.std_radrd_ = std_radrd;
}


WHEN("^I call PredictRadarMeasurement$") {
  int n_aug = 7;
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);

  ScenarioScope<UKFCtx> context;
  context->ukf_.PredictRadarMeasurement(&Zsig, &z_pred, &S);
  context->vectors["z_pred"] = z_pred;
  context->matrix_vars["S"] = S;
}


GIVEN("^std_a_ is (.*)$", (const double std_a)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.std_a_ = std_a;
}


GIVEN("^std_yawdd_ is (.*)$", (const double std_yawdd)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.std_yawdd_ = std_yawdd;
}


GIVEN("^x_ is$", (const std::string x_str)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.x_ = ParseVectorXd(x_str);
}


GIVEN("^P_ is$", (const std::string P_str)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.P_ = ParseMatrixXd(P_str);
}


WHEN("^I call Prediction with (.*)$", (const double delta_t)) {
  ScenarioScope<UKFCtx> context;
  context->ukf_.Prediction(delta_t);
}


THEN("^(.+) should be called$", (const std::string expected_call)) {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->ukf_calls[0], expected_call);
  context->ukf_calls.erase(context->ukf_calls.begin());
}


THEN("^Xsig_pred_ with accuracy (.*) should be$", (const double accuracy,
                                                   const std::string Xsig_pred_str)) {
  MatrixXd expected_Xsig_pred = ParseMatrixXd(Xsig_pred_str);
  ScenarioScope<UKFCtx> context;
  EXPECT_TRUE(context->ukf_.Xsig_pred_.isApprox(expected_Xsig_pred, accuracy));
}


GIVEN("^an?o?t?h?e?r? measurement$", (const std::string line)) {
  ScenarioScope<UKFCtx> context;
  VectorXd gt_values(4);
  ParseMeasurement(line, context->meas_package, &gt_values);
}


WHEN("^I call ProcessMeasurement$") {
  ScenarioScope<UKFCtx> context;
  context->ukf_.ProcessMeasurement(context->meas_package);
}


GIVEN("^I called ProcessMeasurement$") {
  ScenarioScope<UKFCtx> context;
  context->ukf_.ProcessMeasurement(context->meas_package);
}


THEN("^x_ with accuracy (.*) should be$", (const double accuracy,
                                           const std::string x_str)) {
  VectorXd expected_x = ParseVectorXd(x_str);
  ScenarioScope<UKFCtx> context;
  EXPECT_TRUE(context->ukf_.x_.isApprox(expected_x, accuracy));
}


THEN("^P_ with accuracy (.*) should be$", (const double accuracy,
                                           const std::string P_str)) {
  MatrixXd expected_P = ParseMatrixXd(P_str);
  ScenarioScope<UKFCtx> context;
  EXPECT_TRUE(context->ukf_.P_.isApprox(expected_P, accuracy));
}


GIVEN("^It would return matrix (.+) as$", (const std::string variable, 
                                           const std::string value)) {
  MatrixXd return_val = ParseMatrixXd(value);
  ScenarioScope<UKFCtx> context;
  context->matrix_vars[variable] = return_val;
}


GIVEN("^It would return vector (.+) as$", (const std::string variable, 
                                           const std::string value)) {
  VectorXd return_val = ParseVectorXd(value);
  ScenarioScope<UKFCtx> context;
  context->vectors[variable] = return_val;
}


GIVEN("^PredictRadarMeasurement would get called$") {
  ScenarioScope<UKFCtx> context;
  EXPECT_CALL(context->ukf_, PredictRadarMeasurement(_, _, _))
      .WillOnce(Invoke(&(*context), &UKFCtx::UKFPredictRadarMeasurement));
}


GIVEN("^Prediction would get called$") {
  ScenarioScope<UKFCtx> context;
  EXPECT_CALL(context->ukf_, Prediction(_))
      .WillOnce(DoAll(Record(&(context->ukf_calls), "Prediction"),
                      SaveArg<0>(&(context->delta_t)), 
                      Invoke(&(context->ukf_), &MockUKF::UKFPrediction)));
  //Reroute Inner calls without logging;
  EXPECT_CALL(context->ukf_, AugmentedSigmaPoints(_))
      .WillOnce(Invoke(&(context->ukf_), &MockUKF::UKFAugmentedSigmaPoints));
  EXPECT_CALL(context->ukf_, PredictMeanAndCovariance(_, _))
      .WillOnce(Invoke(&(context->ukf_), &MockUKF::UKFPredictMeanAndCovariance));  
}


THEN("^It should have received delta_t as (.*)$", (const double expected_delta_t)) {
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->delta_t, expected_delta_t);
}


GIVEN("^UpdateLidar would get called$") {
  ScenarioScope<UKFCtx> context;
  EXPECT_CALL(context->ukf_, UpdateLidar(_))
      .WillOnce(DoAll(Record(&(context->ukf_calls), "UpdateLidar"),
                      SaveArg<0>(&(context->meas_package_arg)), 
                      Invoke(&(context->ukf_), &MockUKF::UKFUpdateLidar)));
}


THEN("^It should have received measurement in$", (const std::string measurement_str)) {
  MeasurementPackage expected_measurement;
  VectorXd gt_values(4);
  ParseMeasurement(measurement_str, expected_measurement, &gt_values);
  ScenarioScope<UKFCtx> context;
  ASSERT_EQ(context->meas_package_arg, expected_measurement);
}


GIVEN("^UpdateRadar would get called$") {
  ScenarioScope<UKFCtx> context;
  EXPECT_CALL(context->ukf_, UpdateRadar(_))
      .WillOnce(DoAll(Record(&(context->ukf_calls), "UpdateRadar"),
                      SaveArg<0>(&(context->meas_package_arg)), 
                      Invoke(&(context->ukf_), &MockUKF::UKFUpdateRadar)));
}


WHEN("^I call UpdateRadar$") {
  ScenarioScope<UKFCtx> context;
  context->ukf_.UpdateRadar(context->meas_package);
}


GIVEN("^Dataset 1 is used$") {
  ScenarioScope<UKFCtx> context;
  std::string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  std::ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  ASSERT_TRUE(in_file_.is_open());

  std::string line;

  while (getline(in_file_, line)) {
    context->measurements.push_back(line);
  }
  // close files
  if (in_file_.is_open()) {
    in_file_.close();
  }
}
