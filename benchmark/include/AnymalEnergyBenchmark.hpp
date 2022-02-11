//
// Created by kangd on 14.05.18.
//

#ifndef BENCHMARK_ANYMALZEROGBENCHMARK_HPP
#define BENCHMARK_ANYMALZEROGBENCHMARK_HPP

#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "BenchmarkTest.hpp"

/**
 * ANYmal Energy test (free drop) checks energy conservation of articulated system model
 * The test focuses on:
 *
 * 1. Energy conservation vs simulation speed trade-off
 *
 * Please read docs for more details
 */

namespace po = boost::program_options;

namespace benchmark::anymal::freedrop {

/**
 * options for simulation
 */
    struct Option: benchmark::Option {
        double dt = 0.005;  // timestep (sec)
        double guiRealtimeFactor = 0.2;
    };
    Option options;

/**
 * parameter for simulation
 */
    struct Parameter {

        // sim properties
        double lightPosition[3] = {30.0, 0, 10.0};

        double T1 = 1;      // force applying time
        double T2 = 2;      // free drop time
        double H = 2;       // initial height
        double M = 0;       // will be updated! (anymal mass)
        double g = -9.81;
        double F = 0;       // will be updated
    };
    Parameter params;

/**
 * data for simulation
 */
    struct Data {
        void setN(int n) {
          Data::n = n;
          kineticE.reserve(n);
          potentialE.reserve(n);
        }

        double computeError() {
          Eigen::MatrixXd energyErrorSq(n, 1);

          for(int i = 0; i < n; i++) {
            energyErrorSq(i, 0) = pow(kineticE[i] + potentialE[i] - E0, 2);
          }

          return energyErrorSq.mean();
        }

        std::vector<double> kineticE;
        std::vector<double> potentialE;

        // num data
        double E0 = 0;
        int n = 0;
    };
    Data data;

/**
 * get URDF file path of ANYmal
 * (For ODE, Dart and RaiSim)
 *
 * @return urdfPath in string
 */
    std::string getURDFpath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-energy-benchmark/ode-rai-dart/robot.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of plane
 * (For Bullet)
 *
 * @return urdfPath in string
 */
    std::string getBulletPlanePath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-energy-benchmark/bullet/plane.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of ANYmal
 * (For Bullet)
 *
 * @return urdfPath in string
 */
    std::string getBulletANYmalPath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-energy-benchmark/bullet/robot.urdf";

      return urdfPath;
    }

/**
 * get URDF file path of ANYmal
 * (For MuJoCo)
 *
 * @return urdfPath in string
 */
    std::string getMujocoURDFpath() {

      std::string urdfPath(RESOURCEPATH);
      urdfPath += "/benchmark/ANYmal-energy-benchmark/mujoco/robot.urdf";

      return urdfPath;
    }

/**
 * get YAML file path for parameter
 *
 * @return yaml path in string
 */
    std::string getYamlpath() {

      std::string yamlPath(BENCHMARKYAMLPATH);
      yamlPath += "/anymal-energy.yaml";

      return yamlPath;
    }

/**
 * get rlog file path of test result
 *
 * @param feedback
 * @return log file path in string
 */
    std::string getLogDirpath(bool feedback) {

      std::string logPath(DATAPATH);
      logPath += "/anymal-feedrop/";
      return logPath;
    }

    std::string getCSVpath() {

      std::string csvPath(DATAPATH);
      csvPath += "/anymal-energy/" + options.csvName;
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
              ("dt", po::value<double>(), "time step for simulation (e.g. 0.01)")
              ("plot", "plot energy error")
              ;
    }

/**
 * get options from arguments
 *
 * @param argc
 * @param argv
 * @param desc
 */
    void getOptionsFromArg(int argc, const char **argv, po::options_description &desc) {

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

      // nogui option
      if(vm.count("nogui")) {
        options.gui = false;
      }

      // save video
      if(vm.count("video")) {
        RSFATAL_IF(!options.gui, "GUI should be on to save a video")
        options.saveVideo = true;
      }

      // dt
      if(vm.count("dt")) {
        options.dt = vm["dt"].as<double>();
      }

      // csv
      if(vm.count("csv")) {
        options.csv = true;
        options.csvName = vm["csv"].as<std::string>();
      }

      // plot
      if(vm.count("plot")) {
        options.plot = true;
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
      params.g = constant["g"].as<double>();
      params.H = constant["H"].as<double>();
      params.T1 = constant["T1"].as<double>();
      params.T2 = constant["T2"].as<double>();

      // solver parameters
      YAML::Node solver_params = yaml["solver_params"];

      switch (simulator) {
        case benchmark::RAI:
          break;
        case benchmark::BULLET:
          break;
        case benchmark::ODE:
          break;
        case benchmark::MUJOCO:
          break;
        case benchmark::DART:
          break;
        default:
         RSFATAL("invalid simulator value")
      }
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
             << options.dt << ","
             << error << ","
             << time << std::endl;
      myfile.close();
    }

} // benchmark::anymal

#endif //BENCHMARK_ANYMALZEROGBENCHMARK_HPP
