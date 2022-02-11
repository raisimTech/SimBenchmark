//
// Created by kangd on 26.04.18.
//

#include <BtMbSim.hpp>

#include "AtlasContactBenchmark.hpp"
#include "BtMbBenchmark.hpp"


bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> robots;
po::options_description desc;

void setupSimulation() {
  if(benchmark::atlas::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // time step
  sim->setTimeStep(benchmark::atlas::params.dt);
  sim->setERP(0.05, 0.01, 0.1);
}

void resetWorld() {
  auto checkerboard = sim->addArticulatedSystem(benchmark::atlas::getBulletPlanePath(),
                                                bullet_mb_sim::object::URDF,
                                                true,
                                                benchmark::bulletmultibody::options.maximalCoordinate);

  checkerboard->setFrictionCoefficient(-1, 1.0);  // 1.0(ground) x 0.8(feet) = 0.8
  for(int i = 0; i < benchmark::atlas::options.numRow; i++) {
    for(int j = 0; j < benchmark::atlas::options.numRow; j++) {
      auto robot =
          sim->addArticulatedSystem(benchmark::atlas::getBulletAtlasPath(),
                                    bullet_mb_sim::object::URDF,
                                    true,
                                    benchmark::bulletmultibody::options.maximalCoordinate);
//      atlas->setColor({1, 0, 0, 1});

      Eigen::VectorXd gc(robot->getStateDimension());
      Eigen::VectorXd gv(robot->getDOF());
      Eigen::VectorXd tau(robot->getDOF());
      gc.setZero();
      gc.segment<7>(0) << 2.5 * i, 2.5 * j, benchmark::atlas::params.H,
          benchmark::atlas::params.baseQuat[0],
          benchmark::atlas::params.baseQuat[1],
          benchmark::atlas::params.baseQuat[2],
          benchmark::atlas::params.baseQuat[3];

      robot->setGeneralizedCoordinate(gc);
      robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
      robots.push_back(robot);
    }
  }

  sim->setGravity({0, 0, benchmark::atlas::params.g});

  if(benchmark::atlas::options.gui)
    sim->cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});
}

double simulationLoop(bool timer, bool cntNumContact) {
  // resever error vector
  if(cntNumContact)
    benchmark::atlas::data.setN(int(benchmark::atlas::params.T / benchmark::atlas::params.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int t = 0; t < int(benchmark::atlas::params.T / benchmark::atlas::params.dt); t++) {
    if(benchmark::atlas::options.gui && !sim->visualizerLoop(benchmark::atlas::params.dt))
      break;

    for(int i = 0; i < robots.size(); i++) {
      Eigen::VectorXd gc(robots[i]->getStateDimension());
      Eigen::VectorXd gv(robots[i]->getDOF());
      Eigen::VectorXd tau(robots[i]->getDOF());
      gc = robots[i]->getGeneralizedCoordinate();
      gv = robots[i]->getGeneralizedVelocity();
      tau.setZero();
      tau.tail(30) =
          -benchmark::atlas::params.kp.tail(30).cwiseProduct(gc.tail(30))
              - benchmark::atlas::params.kd.cwiseProduct(gv).tail(30);
      robots[i]->setGeneralizedForce(tau);
    }
    sim->integrate();
    if(cntNumContact) benchmark::atlas::data.numContactList.push_back(sim->getWorldNumContacts());
  }

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::atlas::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::atlas::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::atlas::getParamsFromYAML(benchmark::atlas::getYamlpath().c_str(),
                                      benchmark::BULLET);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << benchmark::bulletmultibody::options.simName << std::endl
                << "GUI      : " << benchmark::atlas::options.gui << std::endl
                << "Row      : " << benchmark::atlas::options.numRow << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  resetWorld();
  simulationLoop(false, true);
  double avgNumContacts = benchmark::atlas::data.computeAvgNumContact();

  // reset
  robots.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  resetWorld();
  double time = simulationLoop(true, false);

  // print to screen
  std::cout<<"time taken for "
           << (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt)
           << " steps "<< time <<"s \n";

  if(benchmark::atlas::options.csv)
    benchmark::atlas::printCSV(benchmark::atlas::getCSVpath(),
                               benchmark::bulletmultibody::options.simName,
                               benchmark::bulletmultibody::options.solverName,
                               benchmark::bulletmultibody::options.detectorName,
                               benchmark::bulletmultibody::options.integratorName,
                               benchmark::atlas::options.numRow,
                               avgNumContacts,
                               time);

  delete sim;
  return 0;
}