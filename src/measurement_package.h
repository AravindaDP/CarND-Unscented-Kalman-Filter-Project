#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;

  bool operator==(const MeasurementPackage& rhs) const{
    return ((timestamp_ == rhs.timestamp_) && (sensor_type_ == rhs.sensor_type_) && raw_measurements_.isApprox(rhs.raw_measurements_));
  }
};

#endif // MEASUREMENT_PACKAGE_H_