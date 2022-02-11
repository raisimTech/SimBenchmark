//
// Created by kangd on 26.04.18.
//

#include <BtMbSim.hpp>

#include "AnymalPDBenchmark.hpp"
#include "BtMbBenchmark.hpp"


bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  sim->setERP(0.05, 0.01, 0.1);
  sim->setTimeStep(benchmark::anymal::params.dt);

  int numAnymal = benchmark::anymal::options.numRow * benchmark::anymal::options.numRow;
//  sim->setSolverParameter(1e-7, 1000);
}

void resetWorld() {
  auto checkerboard = sim->addArticulatedSystem(benchmark::anymal::getBulletPlanePath(),
                                                bullet_mb_sim::object::URDF,
                                                true,
                                                benchmark::bulletmultibody::options.maximalCoordinate);
  checkerboard->setFrictionCoefficient(-1, 1.0);  // 1.0(ground) x 0.8(feet) = 0.8

  for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
    for(int j = 0; j < benchmark::anymal::options.numRow; j++) {
      auto anymal =
          sim->addArticulatedSystem(benchmark::anymal::getBulletANYmalPath(),
                                    bullet_mb_sim::object::URDF,
                                    true,
                                    benchmark::bulletmultibody::options.maximalCoordinate);
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
//      anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
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

  if(benchmark::anymal::options.gui) {
    // gui
    while(sim->visualizerLoop(benchmark::anymal::params.dt, 1.0)) {
      if(benchmark::anymal::options.feedback) {
        for (int i = 0; i < anymals.size(); i++) {
          jointState = anymals[i]->getGeneralizedCoordinate();
          jointVel = anymals[i]->getGeneralizedVelocity();
//          jointForce = anymals[i]->getGeneralizedForce();

          jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
          jointForce.head(6).setZero();
          anymals[i]->setGeneralizedForce(jointForce);
        }
      }
      sim->integrate();
    }
  } else {
    // no gui
    StopWatch watch;
    watch.start();
    for(int t = 0; t < (int)(benchmark::anymal::params.T / benchmark::anymal::params.dt); t++) {
      if(benchmark::anymal::options.feedback) {
        for (int i = 0; i < anymals.size(); i++) {
          jointState = anymals[i]->getGeneralizedCoordinate();
          jointVel = anymals[i]->getGeneralizedVelocity();
//          jointForce = anymals[i]->getGeneralizedForce();

          jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
          jointForce.head(6).setZero();
          anymals[i]->setGeneralizedForce(jointForce);
        }
      }
      sim->integrate();
    }

    double time = watch.measure();

    // print to screen
    std::cout<<"time taken for "
             << (int) (benchmark::anymal::params.T / benchmark::anymal::params.dt)
             << " steps "<< time <<"s \n";

    if(benchmark::anymal::options.csv)
      benchmark::anymal::printCSV(benchmark::anymal::getCSVpath(benchmark::anymal::options.feedback),
                                  benchmark::bulletmultibody::options.simName,
                                  benchmark::bulletmultibody::options.solverName,
                                  benchmark::bulletmultibody::options.detectorName,
                                  benchmark::bulletmultibody::options.integratorName,
                                  benchmark::anymal::options.numRow,
                                  time);
  }
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::anymal::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::getParamsFromYAML(benchmark::anymal::getYamlpath().c_str(),
                                       benchmark::BULLET);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << benchmark::bulletmultibody::options.simName << std::endl
                << "GUI      : " << benchmark::anymal::options.gui << std::endl
                << "Row      : " << benchmark::anymal::options.numRow << std::endl
                << "Feedback : " << benchmark::anymal::options.feedback << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "-----------------------"
  )
  setupSimulation();
  resetWorld();

  int profileID = -1;
  if(benchmark::bulletmultibody::options.profiling)
    profileID= sim->startBulletProfiling("/tmp/btmb-anymal-benchmark");
  simulationLoop();
  if(benchmark::bulletmultibody::options.profiling)
    sim->stopBulletProfiling(profileID);

   RSINFO(
      std::endl << "-----------------------" << std::endl
                << "Contacts : " << sim->getWorldNumContacts() << std::endl
                << "======================="
  )

  delete sim;
  return 0;
}