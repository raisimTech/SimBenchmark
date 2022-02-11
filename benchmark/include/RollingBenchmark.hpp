//
// Created by kangd on 17.02.18.
//

#ifndef BENCHMARK_ROLLING_HPP
#define BENCHMARK_ROLLING_HPP

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "BenchmarkTest.hpp"

/**
 * Rolling test investigates accuracy of frictional contact solving.
 * The error is measured by comparing the simulation with analytical solution.
 * The test focuses on:
 *
 * 1. Frictional cone (diagonal, elliptic)
 * 2. The accuracy of frictional contact solving vs simulation speed trade-off
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;

namespace benchmark::rolling {

/// enum
enum ForceDirection {
  FORCE_Y,    // force along the y axis
  FORCE_XY    // force along the diagonal direction
};

/// functions
std::string getMujocoXMLpath();
std::string getBulletBallPath();
std::string getBulletBoxPath();
std::string getBulletPlanePath();
std::string getYamlpath();
std::string getLogDirpath(bool, ForceDirection,
                          std::string, std::string, std::string, std::string, double);
std::string getCSVpath();

void addDescToOption(po::options_description&);
void getOptionsFromArg(int argc, const char *argv[], po::options_description &);
void getParamsFromYAML(const char *yamlfile, benchmark::Simulator simulator);

void loggerSetup(std::string path, std::string name);
void printCSV(std::string, std::string, std::string, std::string, std::string, double, double error);

Eigen::Vector3d computeAnalyticalSol(double t, bool isBall);

/// struct

/**
 * options for rolling simulation
 */
struct Option: benchmark::Option {
  // force-direction Y/XY
  ForceDirection forceDirection = FORCE_XY;

  // erp
  bool erpYN = false;

  // default params
  bool defaultParam = false;

  // time step
  double dt = 0.001;

  /// Note
  /// 1. Bullet, RAI, Mujoco has both solver tolerance and iteration option
  /// 2. ODE has no solver tolerance. (always iterate with numSolverIter value)
  /// 3. Dart cannot set solver parameters
  // num solver iter
  int numSolverIter = 1000;

  // solver tolerance
  double solverTol = 1e-30;
};
Option options;

/**
 * parameter for rolling simulation
 * this can be set by YAML file
 */
struct Parameter {

  // sim properties
  double lightPosition[3] = {30.0, 0, 10.0};

  // solver parameters
  double erp = 0.2;
  double erp2 = 0.1;            // for bullet
  double erpFriction = 0.1;     // for bullet

  // simulation parameters
  double m = 1;
  int n = 5;     // num balls = n x n
  double M = 10;
  double g = -9.81;
  double T = 4.0;
  double F = 150;

  double initPenetration = 5e-6;

  double btGroundMu = 0.5;
  double btBallMu = 1.0;
  double btBoxMu = 0.8;

  double odeGroundMu = 0.5;
  double odeBallMu = 1.0;
  double odeBoxMu = 0.8;

  double raiGroundMu = 0.5;
  double raiBallMu = 1.0;
  double raiBoxMu = 0.8;

  double dartGroundMu = 0.4;
  double dartBallMu = 0.8;
  double dartBoxMu = 0.8;

  double mjcGroundMu = 0.4;
  double mjcBallMu = 0.8;
  double mjcBoxMu = 0.4;

  /// note
  /// 1. (frictional coeff A-B) = (friction coeff of A) x (friction coeff of B)             - Bullet & ODE
  /// 2. (frictional coeff A-B) = max of (friction coeff of A) and (friction coeff of B)    - Mujoco
  /// 3. (frictional coeff A-B) = min of (friction coeff of A) and (friction coeff of B)    - Dart
  /// 4. (frictional coeff A-B)                                                             - Rai
};
Parameter params;

/**
 * data from rolling simulation
 */
struct Data {
  void setN(int n) {
    Data::n = n;
    ballVel.reserve(n);
    ballPos.reserve(n);
    boxVel.reserve(n);
    boxPos.reserve(n);
  }

  /**
   * compute sum of mean squared error box and mean squared error ball
   * @return
   */
  double computeError() {
    Eigen::MatrixXd velErrorSq(n, 3);
    for(int i = 0; i < n; i++) {
      Eigen::Vector3d ballVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      velErrorSq.row(i).x() = pow((ballVel[i].x() - ballVec.x()), 2) + pow((boxVel[i].x() - boxVec.x()), 2);
      velErrorSq.row(i).y() = pow((ballVel[i].y() - ballVec.y()), 2) + pow((boxVel[i].y() - boxVec.y()), 2);
      velErrorSq.row(i).z() = pow((ballVel[i].z() - ballVec.z()), 2) + pow((boxVel[i].z() - boxVec.z()), 2);
    }

    return velErrorSq.rowwise().sum().mean();
  }

  // data list
  std::vector<Eigen::Vector3d> ballVel;
  std::vector<Eigen::Vector3d> ballPos;
  std::vector<Eigen::Vector3d> boxVel;
  std::vector<Eigen::Vector3d> boxPos;

  // num data
  int n = 0;
};
Data data;

/**
 * get XML file path for Mujoco
 *
 * @param rowNum # of row
 * @return urdf path in string
 */
std::string getMujocoXMLpath() {

  std::string xmlPath(RESOURCEPATH);
  xmlPath += "/benchmark/rolling-benchmark/mujoco/rolling.xml";

  return xmlPath;
}

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
std::string getBulletBallPath() {

  std::string ballpath(RESOURCEPATH);
  ballpath += "/benchmark/rolling-benchmark/bullet/ball.urdf";

  return ballpath;
}

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
std::string getBulletBoxPath() {

  std::string ballpath(RESOURCEPATH);
  ballpath += "/benchmark/rolling-benchmark/bullet/box.urdf";

  return ballpath;
}

/**
 * get XML file path for Mujoco
 *
 * @return urdf path in string
 */
std::string getBulletPlanePath() {

  std::string planepath(RESOURCEPATH);
  planepath += "/benchmark/rolling-benchmark/bullet/plane.urdf";

  return planepath;
}

/**
 * get YAML file path for parameter
 *
 * @return yaml path in string
 */
std::string getYamlpath() {

  std::string yamlPath(BENCHMARKYAMLPATH);
  yamlPath += "/rolling.yaml";

  return yamlPath;
}

/**
 * get log file directory path of test result
 *
 * @param erp
 * @return log directory path in string
 */
std::string getLogDirpath(bool erpYN,
                          ForceDirection forceDirection,
                          std::string simulation,
                          std::string solver,
                          std::string detector,
                          std::string integrator,
                          double dt) {

  std::string dirPath(DATAPATH);

  dirPath += "/rolling-benchmark/erp=" + std::to_string(erpYN)
      + "-dir=" + std::to_string(forceDirection)
      + "/" + simulation
      + "/" + solver
      + "/" + detector
      + "/" + integrator
      + "/" + std::to_string(dt);

  return dirPath;
}

std::string getCSVpath() {

  std::string csvPath(DATAPATH);
  csvPath += "/rolling/" + options.csvName;

  return csvPath;
}

/**
 * add options to desc
 *
 * @param desc
 */
void addDescToOption(po::options_description &desc) {
  benchmark::addDescToOption(desc);
  desc.add_options()
      ("erp-on", po::value<bool>(), "erp on (true / false)")
      ("plot", "plot on")
      ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
      ("force", po::value<std::string>(), "applied force direction (y / xy)")
      ("numiter", po::value<int>(), "the number of solver iteration or max number of iteration. (set default number if this option is not set)")
      ("tolerance", po::value<double>(), "solver tolerance value. (set default number if this option is not set)")
      ("default", "test with default solver parameters")
      ;
}

/**
 * get options from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
void getOptionsFromArg(int argc, const char *argv[], po::options_description &desc) {

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  // help option
  if(vm.count("help")) {
    std::cout << desc << std::endl;
    exit(0);
  }

  // log option
  if(vm.count("log")) {
    options.log = true;
  }

  // log option
  if(vm.count("plot")) {
    options.plot = true;
  }

  // nogui option
  if(vm.count("nogui")) {
    options.gui = false;
  }

  // save video
  if(vm.count("video")) {
     RSFATAL_IF(!options.gui, "GUI should be on to save a video")
    options.saveVideo = true;
  }

  // dt option
  if(vm.count("dt")) {
    options.dt = vm["dt"].as<double>();
  }

  // force direction
  if(vm.count("force")) {
    if(vm["force"].as<std::string>().compare("xy") == 0) {
      options.forceDirection = FORCE_XY;
    } else if(vm["force"].as<std::string>().compare("y") == 0) {
      options.forceDirection = FORCE_Y;
    } else {
       RSFATAL("invalid force input (should be xy or y)")
    }
  }

  // erp
  if(vm.count("erp-on")) {
    if(vm["erp-on"].as<bool>()) {
      options.erpYN = true;
    } else {
      options.erpYN = false;
    }
  }

  // csv
  if(vm.count("csv")) {
    options.csv = true;
    options.csvName = vm["csv"].as<std::string>();
  }

  // num iter
  if(vm.count("numiter")) {
    options.numSolverIter = vm["numiter"].as<int>();
  }

  // tolerance
  if(vm.count("tolerance")) {
    options.solverTol = vm["tolerance"].as<double>();
  }

  // tolerance
  if(vm.count("default")) {
    options.defaultParam = true;
  }
}

/**
 * get params from YAML
 *
 * @param yamlfile
 */
void getParamsFromYAML(const char *yamlfile, benchmark::Simulator simulator) {
  /// parameters from yaml
  YAML::Node yaml = YAML::LoadFile(yamlfile);

  // sim properties
  YAML::Node props = yaml["sim_properties"];
  params.lightPosition[0] = props["light_position"].as<std::vector<double>>()[0];
  params.lightPosition[1] = props["light_position"].as<std::vector<double>>()[1];
  params.lightPosition[2] = props["light_position"].as<std::vector<double>>()[2];

  // simulation constants
  YAML::Node constant = yaml["constant"];
  params.m = constant["m"].as<double>();
  params.n = constant["n"].as<int>();
  params.M = constant["M"].as<double>();
  params.g = constant["g"].as<double>();
  params.T = constant["T"].as<double>();
  params.F = constant["F"].as<double>();
  params.initPenetration = constant["penentration0"].as<double>();

  // solver parameters
  YAML::Node solver_params = yaml["solver_params"];

  switch (simulator) {
    case benchmark::RAI:
      params.erp = solver_params["raiSim"]["erp"].as<double>();
      params.raiGroundMu = constant["raiSim"]["mu_ground"].as<double>();
      params.raiBallMu = constant["raiSim"]["mu_ball"].as<double>();
      params.raiBoxMu = constant["raiSim"]["mu_box"].as<double>();
      break;
    case benchmark::BULLET:
      params.erp = solver_params["bullet"]["erp"].as<double>();
      params.erp2 = solver_params["bullet"]["erp2"].as<double>();
      params.erpFriction = solver_params["bullet"]["erp_friction"].as<double>();
      params.btGroundMu = constant["bullet"]["mu_ground"].as<double>();
      params.btBallMu = constant["bullet"]["mu_ball"].as<double>();
      params.btBoxMu = constant["bullet"]["mu_box"].as<double>();
      break;
    case benchmark::ODE:
      params.erp = solver_params["ode"]["erp"].as<double>();
      params.odeGroundMu = constant["ode"]["mu_ground"].as<double>();
      params.odeBallMu = constant["ode"]["mu_ball"].as<double>();
      params.odeBoxMu = constant["ode"]["mu_box"].as<double>();
      break;
    case benchmark::MUJOCO:
      params.mjcGroundMu = constant["mujoco"]["mu_ground"].as<double>();
      params.mjcBallMu = constant["mujoco"]["mu_ball"].as<double>();
      params.mjcBoxMu = constant["mujoco"]["mu_box"].as<double>();
      break;
    case benchmark::DART:
      params.dartGroundMu = constant["dart"]["mu_ground"].as<double>();
      params.dartBallMu = constant["dart"]["mu_ball"].as<double>();
      params.dartBoxMu = constant["dart"]["mu_box"].as<double>();
      break;
    default:
     RSFATAL("invalid simulator value")
  }
}

Eigen::Vector3d computeAnalyticalSol(double t, bool isBall) {
  const double g = -benchmark::rolling::params.g;
  const double m = benchmark::rolling::params.m;
  const double M = benchmark::rolling::params.M;
  const double F = benchmark::rolling::params.F;
  const int n = benchmark::rolling::params.n * benchmark::rolling::params.n;
  const double mu1 = benchmark::rolling::params.raiGroundMu * benchmark::rolling::params.raiBoxMu;
  const double mu2 = benchmark::rolling::params.raiBoxMu * benchmark::rolling::params.raiBallMu;

  const double simTime = benchmark::rolling::params.T;
  const double dt = benchmark::rolling::options.dt;

  double f1 = mu1 * (M + n * m)  * g;
  double f2 = 1 / M * (150 - f1) / (3.5 / m + 25 / M);
  double a1 = (F - f1 - n * f2) / M;
  double a2 = f2 / m;

  double v = 0;
  if (isBall)
    v = a2 * t;
  else
    v = a1 * t;

  if (benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
//    return {v * 0.707106781186547, v * 0.707106781186547, 0};
    return {v * 0.5, v * 0.866025403784439, 0};
  else
    return {0, v, 0};
}

void printCSV(std::string filePath,
              std::string sim,
              std::string solver,
              std::string detector,
              std::string integrator,
              double time,
              double error) {
  std::ofstream myfile;
  myfile.open (filePath, std::ios_base::app);

  myfile << sim << ","
         << solver << ","
         << detector << ","
         << integrator << ","
         << options.erpYN << ","
         << options.forceDirection << ","
         << options.dt << ","
         << options.numSolverIter << ","
         << options.solverTol << ","
         << error << ","
         << time << std::endl;
  myfile.close();
}

} // benchmark::rolling-benchmark

#endif //BENCHMARK_ROLLING_HPP
