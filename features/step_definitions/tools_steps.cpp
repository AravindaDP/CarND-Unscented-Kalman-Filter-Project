#include <gtest/gtest.h>
#include <cucumber-cpp/autodetect.hpp>
#include "../../src/tools.h"
#include <vector>

using cucumber::ScenarioScope;
using Eigen::Map;
using Eigen::VectorXd;

struct ToolsCtx {
  Tools tools;
  VectorXd result;
  std::vector<VectorXd> estimations;
  std::vector<VectorXd> ground_truth;
};

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

GIVEN("^estimations length is 0$") {
  ScenarioScope<ToolsCtx> context;
  context->estimations.clear();
}


GIVEN("^ground_truth length is 0$") {
  ScenarioScope<ToolsCtx> context;
  context->estimations.clear();
}


GIVEN("^estimations is$") {
  REGEX_PARAM(std::string, estimations_str);
  ScenarioScope<ToolsCtx> context;
  context->estimations = ParseVectorOfVectorXd(estimations_str);
}


GIVEN("^ground_truth is$") {
  REGEX_PARAM(std::string, ground_truth_str);
  ScenarioScope<ToolsCtx> context;
  context->ground_truth = ParseVectorOfVectorXd(ground_truth_str);
}


WHEN("^I calculate RMSE$") {
  ScenarioScope<ToolsCtx> context;
  context->result = context->tools.CalculateRMSE(context->estimations, context->ground_truth);
}


THEN("^I should get \"([^\"]*)\" as output$") {
  REGEX_PARAM(std::string, output_str);
  VectorXd expected_rmse = ParseVectorXd(output_str);
  ScenarioScope<ToolsCtx> context;
  EXPECT_TRUE(context->result.isApprox(expected_rmse));
}