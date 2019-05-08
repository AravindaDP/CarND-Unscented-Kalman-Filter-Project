#include <gtest/gtest.h>
#include <cucumber-cpp/autodetect.hpp>
#include "../../src/tools.h"
#include <vector>
#include "helpers.h"

using cucumber::ScenarioScope;
using Eigen::Map;
using Eigen::VectorXd;

struct ToolsCtx {
  Tools tools;
  VectorXd result;
  std::vector<VectorXd> estimations;
  std::vector<VectorXd> ground_truth;
};


GIVEN("^estimations length is 0$") {
  ScenarioScope<ToolsCtx> context;
  context->estimations.clear();
}


GIVEN("^ground_truth length is 0$") {
  ScenarioScope<ToolsCtx> context;
  context->estimations.clear();
}


GIVEN("^estimations is$", (const std::string estimations_str)) {
  ScenarioScope<ToolsCtx> context;
  context->estimations = ParseVectorOfVectorXd(estimations_str);
}


GIVEN("^ground_truth is$", (const std::string ground_truth_str)) {
  ScenarioScope<ToolsCtx> context;
  context->ground_truth = ParseVectorOfVectorXd(ground_truth_str);
}


WHEN("^I calculate RMSE$") {
  ScenarioScope<ToolsCtx> context;
  context->result = context->tools.CalculateRMSE(context->estimations, context->ground_truth);
}


THEN("^I should get \"([^\"]*)\" as output$", (const std::string output_str)) {
  VectorXd expected_rmse = ParseVectorXd(output_str);
  ScenarioScope<ToolsCtx> context;
  EXPECT_TRUE(context->result.isApprox(expected_rmse));
}