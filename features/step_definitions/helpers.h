#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>
#include "../../src/Eigen/Dense"
#include "../../src/measurement_package.h"

using Eigen::Map;
using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd ParseVectorXd(const std::string& vector_str);

std::vector<VectorXd> ParseVectorOfVectorXd(const std::string& input);

#endif  // HELPERS_H_