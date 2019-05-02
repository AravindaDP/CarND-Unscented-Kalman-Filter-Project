#include "helpers.h"
#include <sstream>
#include <iterator>

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
