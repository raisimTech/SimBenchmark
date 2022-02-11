//
// Created by kangd on 14.05.18.
//

#include "raisim/World.hpp"

#include "AnymalMomentumBenchmark.hpp"

raisim::World* sim;
std::vector<raisim::SingleBodyObject*> balls;
std::vector<raisim::ArticulatedSystem*> anymals;
po::options_description desc;

void setupSimulation() {
  sim = new raisim::World();

  // set erp 0
  sim->setERP(0);

  // time step
  sim->setTimeStep(benchmark::anymal::zerogravity::options.dt);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addGround();

  // ball
  auto ball = sim->addSphere(0.2, benchmark::anymal::zerogravity::params.m);
  ball->setPosition(0,
                    benchmark::anymal::zerogravity::params.x0,
                    benchmark::anymal::zerogravity::params.H);
  ball->setVelocity(0, benchmark::anymal::zerogravity::params.v0, 0, 0, 0, 0);
  balls.push_back(ball);

  // anymal
  auto anymal = sim->addArticulatedSystem(
      benchmark::anymal::zerogravity::getURDFpath()
  );
  anymal->setGeneralizedCoordinate({0,
                                    0,
                                    benchmark::anymal::zerogravity::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    -0.03, 0.4, -0.8,
                                    0.03, -0.4, 0.8,
                                    -0.03, -0.4, 0.8});
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymals.push_back(anymal);

  // gravity
  sim->setGravity({0, 0, 0});

  // mass
  benchmark::anymal::zerogravity::params.M =
      std::accumulate( anymal->getMass().begin(), anymal->getMass().end(), 0.0);
}

double computeLinearMomentumError() {
  // compute linear momentum
  Eigen::Vector3d linearMomentum;
  linearMomentum.setZero();

  for(int i = 0; i < anymals.size(); i++) {
    linearMomentum += anymals[i]->getLinearMomentum().e();
  }
  for(int i = 0; i < balls.size(); i++) {
    linearMomentum += balls[i]->getLinearMomentum();
  }

  Eigen::Vector3d analyticSol(0, benchmark::anymal::zerogravity::params.m * benchmark::anymal::zerogravity::params.v0, 0);

  return pow((linearMomentum - analyticSol).norm(), 2);
}

double simulationLoop(bool timer = true, bool error = true) {

  // resever error vector
  if(error)
    benchmark::anymal::zerogravity::data.setN(
        unsigned(benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt)
    );

  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  for(int i = 0; i < (int) (benchmark::anymal::zerogravity::params.T / benchmark::anymal::zerogravity::options.dt); i++) {
    // step1
    sim->integrate1();

    // data save
    if(error) {
      benchmark::anymal::zerogravity::data.ballMomentum.push_back(
          balls[0]->getLinearMomentum()
      );
      benchmark::anymal::zerogravity::data.anymalMomentum.push_back(
          anymals[0]->getLinearMomentum().e()
      );
    }

    // step
    sim->integrate2();
  }

  end = std::chrono::steady_clock::now();
  return double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::zerogravity::addDescToOption(desc);
  benchmark::anymal::zerogravity::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::zerogravity::getParamsFromYAML(benchmark::anymal::zerogravity::getYamlpath().c_str(),
                                                    benchmark::RAI);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::anymal::zerogravity::options.gui << std::endl
                << "Timestep : " << benchmark::anymal::zerogravity::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::zerogravity::data.computeError();

  // reset
  balls.clear();
  anymals.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::anymal::zerogravity::options.csv)
    benchmark::anymal::zerogravity::printCSV(benchmark::anymal::zerogravity::getCSVpath(),
                                             "RAI",
                                             "RAI",
                                             "RAI",
                                             "RAI",
                                             time,
                                             error);

   RSINFO(
      std::endl << "CPU Timer : " << time << std::endl
                << "Mean Error: " << error << std::endl
                << "======================="
  )

  delete sim;
  return 0;
}