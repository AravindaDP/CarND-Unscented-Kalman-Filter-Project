#include "gmock/gmock.h"  // Brings in Google Mock.
#include "../src/ukf.h"

using ::testing::_;
using ::testing::Invoke;

class MockUKF : public UKF {
 public:
  MockUKF() {
    // By default, all calls are delegated to the real object.
    ON_CALL(*this, Prediction(_))
        .WillByDefault(Invoke(this, &MockUKF::UKFPrediction));
    ON_CALL(*this, UpdateLidar(_))
        .WillByDefault(Invoke(this, &MockUKF::UKFUpdateLidar));
    ON_CALL(*this, UpdateRadar(_))
        .WillByDefault(Invoke(this, &MockUKF::UKFUpdateRadar));
    ON_CALL(*this, AugmentedSigmaPoints(_))
        .WillByDefault(Invoke(this, &MockUKF::UKFAugmentedSigmaPoints));
    ON_CALL(*this, PredictMeanAndCovariance(_, _))
        .WillByDefault(Invoke(this, &MockUKF::UKFPredictMeanAndCovariance));
    ON_CALL(*this, PredictRadarMeasurement(_, _, _))
        .WillByDefault(Invoke(this, &MockUKF::UKFPredictRadarMeasurement));
  }

  MOCK_METHOD1(Prediction, void(double delta_t));
  MOCK_METHOD1(UpdateLidar, void(MeasurementPackage meas_package));
  MOCK_METHOD1(UpdateRadar, void(MeasurementPackage meas_package));
  MOCK_METHOD1(AugmentedSigmaPoints, void(Eigen::MatrixXd* Xsig_out));
  MOCK_METHOD2(PredictMeanAndCovariance, void(Eigen::VectorXd* x_pred,
                                              Eigen::MatrixXd* P_pred));
  MOCK_METHOD3(PredictRadarMeasurement, void(Eigen::MatrixXd* Zsig_out,
                                             Eigen::VectorXd* z_out, 
                                             Eigen::MatrixXd* S_out));

  // Use this to call Prediction() defined in UKF.
  void UKFPrediction(double delta_t){
    return UKF::Prediction(delta_t);
  }

  // Use this to call UpdateLidar() defined in UKF.
  void UKFUpdateLidar(MeasurementPackage meas_package){
    return UKF::UpdateLidar(meas_package);
  }

  // Use this to call UpdateRadar() defined in UKF.
  void UKFUpdateRadar(MeasurementPackage meas_package){
    return UKF::UpdateRadar(meas_package);
  }

  // Use this to call AugmentedSigmaPoints() defined in UKF.
  void UKFAugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out) {
    return UKF::AugmentedSigmaPoints(Xsig_out);
  }

  // Use this to call PredictMeanAndCovariance() defined in UKF.
  void UKFPredictMeanAndCovariance(Eigen::VectorXd* x_pred, Eigen::MatrixXd* P_pred) {
    return UKF::PredictMeanAndCovariance(x_pred, P_pred);
  }

  // Use this to call PredictRadarMeasurement() defined in UKF.
  void UKFPredictRadarMeasurement(Eigen::MatrixXd* Zsig_out,
                                  Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out) {
    return UKF::PredictRadarMeasurement(Zsig_out, z_out, S_out);
  }
};