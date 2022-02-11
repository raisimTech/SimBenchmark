//
// Created by kangd on 26.04.18.
//

#include "raisim/World.hpp"

#include "AnymalPDBenchmark.hpp"


raisim::World* sim;
std::vector<raisim::ArticulatedSystem*> anymals;
po::options_description desc;

void setupSimulation() {
  sim = new raisim::World();

  // time step
  sim->setTimeStep(benchmark::anymal::params.dt);
}

void resetWorld() {
  auto checkerboard = sim->addGround();

  for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
    for(int j = 0; j < benchmark::anymal::options.numRow; j++) {
      auto anymal = sim->addArticulatedSystem(
          benchmark::anymal::getURDFpath()
      );
      anymal->setGeneralizedCoordinate(
          {i * 2.0,
           j * 2.0,
           benchmark::anymal::params.H,
           benchmark::anymal::params.baseQuat[0],
           benchmark::anymal::params.baseQuat[1],
           benchmark::anymal::params.baseQuat[2],
           benchmark::anymal::params.baseQuat[3],
           benchmark::anymal::params.jointPos[0],
           benchmark::anymal::params.jointPos[1],
           benchmark::anymal::params.jointPos[2],
           benchmark::anymal::params.jointPos[3],
           benchmark::anymal::params.jointPos[4],
           benchmark::anymal::params.jointPos[5],
           benchmark::anymal::params.jointPos[6],
           benchmark::anymal::params.jointPos[7],
           benchmark::anymal::params.jointPos[8],
           benchmark::anymal::params.jointPos[9],
           benchmark::anymal::params.jointPos[10],
           benchmark::anymal::params.jointPos[11]
          });
      anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
      anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
      anymals.push_back(anymal);
    }
  }

  sim->setGravity({0, 0, benchmark::anymal::params.g});
}

void simulationLoop() {
  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
  const double kp = benchmark::anymal::params.kp;
  const double kd = benchmark::anymal::params.kd;

  jointNominalConfig
      <<
      0,
      0,
      benchmark::anymal::params.H,
      benchmark::anymal::params.baseQuat[0],
      benchmark::anymal::params.baseQuat[1],
      benchmark::anymal::params.baseQuat[2],
      benchmark::anymal::params.baseQuat[3],
      benchmark::anymal::params.jointPos[0],
      benchmark::anymal::params.jointPos[1],
      benchmark::anymal::params.jointPos[2],
      benchmark::anymal::params.jointPos[3],
      benchmark::anymal::params.jointPos[4],
      benchmark::anymal::params.jointPos[5],
      benchmark::anymal::params.jointPos[6],
      benchmark::anymal::params.jointPos[7],
      benchmark::anymal::params.jointPos[8],
      benchmark::anymal::params.jointPos[9],
      benchmark::anymal::params.jointPos[10],
      benchmark::anymal::params.jointPos[11];

  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  for(int t = 0; t < (int) (benchmark::anymal::params.T / benchmark::anymal::params.dt); t++) {
    for(int i = 0; i < anymals.size(); i++) {
      jointState = anymals[i]->getGeneralizedCoordinate().e();
      jointVel = anymals[i]->getGeneralizedVelocity().e();
      jointForce = anymals[i]->getGeneralizedForce().e();

      jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
      jointForce.head(6).setZero();
      anymals[i]->setGeneralizedForce(jointForce);
    }
    sim->integrate();
  }

  end = std::chrono::steady_clock::now();

  // print to screen
  std::cout<<"time taken for "
           << (int) (benchmark::anymal::params.T / benchmark::anymal::params.dt)
           << " steps "<< double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6 <<"s \n";

  if(benchmark::anymal::options.csv)
    benchmark::anymal::printCSV(benchmark::anymal::getCSVpath(benchmark::anymal::options.feedback),
                                "RAI",
                                "RAI",
                                "RAI",
                                "RAI",
                                benchmark::anymal::options.numRow,
                                double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6);
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::addDescToOption(desc);
  benchmark::anymal::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::getParamsFromYAML(benchmark::anymal::getYamlpath().c_str(),
                                       benchmark::RAI);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::anymal::options.gui << std::endl
                << "Row      : " << benchmark::anymal::options.numRow << std::endl
                << "Feedback : " << benchmark::anymal::options.feedback << std::endl
                << "-----------------------"
  )

  setupSimulation();
  resetWorld();
  simulationLoop();

   RSINFO(
      std::endl << "=======================" 
  )

  delete sim;
  return 0;
}