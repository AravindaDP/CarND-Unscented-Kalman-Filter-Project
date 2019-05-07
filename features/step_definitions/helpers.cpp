#include "helpers.h"
#include <sstream>
#include <iterator>
#include <iostream>

VectorXd ParseVectorXd(const std::string& vector_str){
  std::istringstream iss(vector_str);
  std::vector<std::string> vector_s(std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>());
  std::vector<double> vector_d;
  for_each(vector_s.begin(), vector_s.end(),
           [&vector_d](const std::string &ele) { vector_d.push_back(stod(ele)); });
  return Map<VectorXd>(vector_d.data(), vector_d.size());
}

std::vector<VectorXd> ParseVectorOfVectorXd(const std::string& input){
  std::vector<VectorXd> result;
  std::string vector_str;
  std::istringstream iss(input);
  while (std::getline(iss, vector_str))
  {
    result.push_back(ParseVectorXd(vector_str));
  }
  return result;
}

MatrixXd ParseMatrixXd(const std::string& input){
  std::vector<VectorXd> rows = ParseVectorOfVectorXd(input);
  MatrixXd eMatrix(rows.size(), rows[0].size());
  for (int i = 0; i < rows.size(); ++i)
    eMatrix.row(i) = rows[i];
  return eMatrix;
}

void ParseMeasurement(const std::string& line, MeasurementPackage& meas_package, 
                      VectorXd* gt){
  std::string sensor_type;
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

  *gt = gt_values;
}
