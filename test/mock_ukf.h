#ifndef MOCK_UKF_H
#define MOCK_UKF_H

#include "gmock/gmock.h"  // Brings in Google Mock.
#include "../src/ukf.h"

using ::testing::_;
using ::testing::Invoke;

class MockUKF : public UKF {
 public:
  MockUKF() {
    // By default, all calls are delegated to the real object.
    ON_CALL(*this, AugmentedSigmaPoints(_))
        .WillByDefault(Invoke(this, &MockUKF::UKFAugmentedSigmaPoints));
    ON_CALL(*this, PredictMeanAndCovariance(_, _))
        .WillByDefault(Invoke(this, &MockUKF::UKFPredictMeanAndCovariance));
    ON_CALL(*this, PredictRadarMeasurement(_, _, _))
        .WillByDefault(Invoke(this, &MockUKF::UKFPredictRadarMeasurement));
  }
  MOCK_METHOD1(AugmentedSigmaPoints, void(Eigen::MatrixXd* Xsig_out));
  MOCK_METHOD2(PredictMeanAndCovariance, void(Eigen::VectorXd* x_pred,
                                              Eigen::MatrixXd* P_pred));
  MOCK_METHOD3(PredictRadarMeasurement, void(Eigen::MatrixXd* Zsig_out,
                                             Eigen::VectorXd* z_out, 
                                             Eigen::MatrixXd* S_out));

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

#endif  // UKF_H