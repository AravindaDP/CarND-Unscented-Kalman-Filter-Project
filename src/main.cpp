#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "ukf.h"
#include "tools.h"
#include "matplotlibcpp.h"
#include <numeric>
#include <string.h>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::ifstream;

// for convenience
using json = nlohmann::json;
namespace plt = matplotlibcpp;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void parseMeasurement(const std::string& line, MeasurementPackage& meas_package, 
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

double run(UKF& ukf, vector<double>& p){
  ukf.std_a_ = p[0];
  ukf.std_yawdd_ = p[1];
  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";

  std::ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  std::string line;

  Tools tools;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    MeasurementPackage meas_package;
    VectorXd gt_values(4);
    parseMeasurement(line, meas_package, &gt_values);

    ground_truth.push_back(gt_values);

    ukf.ProcessMeasurement(meas_package);

    // Push the current estimated x,y positon from the Kalman filter's 
    //   state vector

    VectorXd estimate(4);

    double p_x = ukf.x_(0);
    double p_y = ukf.x_(1);
    double v   = ukf.x_(2);
    double yaw = ukf.x_(3);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);
  }
  // compute the accuracy (RMSE)
  VectorXd  rmse = tools.CalculateRMSE(estimations, ground_truth);

  // close files
  if (in_file_.is_open()) {
    in_file_.close();
  }

  double err = 0;
  for(int i = 0; i < rmse.size(); i++){
      err += rmse(i)*rmse(i);
  }
  return sqrt(err);
}

vector<double> twiddle(double tol=0.02){
  vector<double> p = {3, 1};
  vector<double> dp = {0.5, 0.2};
  UKF ukf;
  double best_err = run(ukf, p);

  int it = 0;
  while(dp[0] + dp[1] > tol) {
    std::cout << "Iteration " << it << ", best error = " << best_err << std::endl;
    for(int i = 0; i< p.size(); i++) {
      p[i] += dp[i];
      UKF ukf;
      double err = run(ukf, p);

      if (err < best_err){
        best_err = err;
        dp[i] *= 1.1;
      }
      else {
        p[i] -= 2 * dp[i];
        p[i] = std::max(0.1, p[i]);
        UKF ukf;
        err = run(ukf, p);

        if (err < best_err){
          best_err = err;
          dp[i] *= 1.1;
        }
        else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
    it += 1;
  }
  return p;
}

int main(int argc, char *argv[]) {
  std::cout << "usage: UnscentedKF [--twiddle|--lidar-only|--radar-only]" << std::endl;
  std::cout << "  options:" << std::endl;
  std::cout << "    --twiddle     Twiddle parameter tuning" << std::endl;
  std::cout << "    --lidar-only  Use Lidar only for measurement updates" << std::endl;
  std::cout << "    --radar-only  Use Radar only for measurement updates" << std::endl;

  if(argc > 1){
    if(strcmp(argv[1], "--twiddle") == 0){
      vector<double> best_params = twiddle();
      std::cout << "Final parameters: " << best_params[0] << "," << best_params[1] << std::endl;
      return 0;
    }
  }

  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  if(argc > 1){
    for(int i = 1; i<argc; i++){
      if(strcmp(argv[i], "--lidar-only") == 0){
        ukf.use_radar_ = false;
      }
      if(strcmp(argv[i], "--radar-only") == 0){
        ukf.use_laser_ = false;
      }
    }
  }

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  vector<double> lidar_nis;
  vector<double> radar_nis;

  h.onMessage([&ukf,&tools,&estimations,&ground_truth, &lidar_nis, &radar_nis]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurement);
          
          VectorXd gt_values(4);

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          parseMeasurement(sensor_measurement, meas_package, &gt_values);

          ground_truth.push_back(gt_values);
          
          // Call ProcessMeasurement(meas_package) for Kalman filter
          ukf.ProcessMeasurement(meas_package);       

          // Push the current estimated x,y positon from the Kalman filter's 
          //   state vector

          VectorXd estimate(4);

          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v   = ukf.x_(2);
          double yaw = ukf.x_(3);

          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
        
          estimations.push_back(estimate);
          if ((sensor_type.compare("L") == 0) && ukf.use_laser_)
            lidar_nis.push_back(ukf.lidar_nis_);
          if ((sensor_type.compare("R") == 0) && ukf.use_radar_)
            radar_nis.push_back(ukf.radar_nis_);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h, &lidar_nis, &radar_nis](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    if(lidar_nis.size()> 0){
      plt::plot(lidar_nis, "r.");
      for(int i=0; i<lidar_nis.size(); ++i) {
        lidar_nis.at(i) = 0.103;
      }
      plt::plot(lidar_nis, "y--");
      for(int i=0; i<lidar_nis.size(); ++i) {
        lidar_nis.at(i) = 5.991;
      }
      plt::plot(lidar_nis, "y--");
      plt::title("Lidar NIS");
      plt::save("../images/lidar_nis.png");
      plt::clf();
      lidar_nis.clear();
    }
    if(radar_nis.size()> 0){
      plt::plot(radar_nis, "r.");
      for(int i=0; i<radar_nis.size(); ++i) {
        radar_nis.at(i) = 0.352;
      }
      plt::plot(radar_nis, "y--");
      for(int i=0; i<radar_nis.size(); ++i) {
        radar_nis.at(i) = 7.815;
      }
      plt::plot(radar_nis, "y--");
      plt::title("Radar NIS");
      plt::save("../images/radar_nis.png");
      radar_nis.clear();
    }
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}