//
// Created by kangd on 26.04.18.
//

#include "raisim/World.hpp"

#include "AtlasContactBenchmark.hpp"


raisim::World* sim;
std::vector<raisim::ArticulatedSystem*> robots;
po::options_description desc;

void setupSimulation() {
  sim = new raisim::World();

  // time step
  sim->setTimeStep(benchmark::atlas::params.dt);
//  sim->setERP(0.01);
//  si1m->setContactSolverParam(1.0, 0.7, 1.0,
//                             50,
//                             1e-7);
}

void resetWorld() {
  auto checkerboard = sim->addGround();

  for(int i = 0; i < benchmark::atlas::options.numRow; i++) {
    for(int j = 0; j < benchmark::atlas::options.numRow; j++) {
      auto robot = sim->addArticulatedSystem(
          benchmark::atlas::getURDFpath()
      );
//      atlas->setColor({1, 0, 0, 1});

      Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim());
      Eigen::VectorXd gv(robot->getDOF());
      Eigen::VectorXd tau(robot->getDOF());
      gc.setZero();
      gv.setZero();
      tau.setZero();
      gc.segment<7>(0) << i * 2.5, j * 2.5, benchmark::atlas::params.H,
          benchmark::atlas::params.baseQuat[0],
          benchmark::atlas::params.baseQuat[1],
          benchmark::atlas::params.baseQuat[2],
          benchmark::atlas::params.baseQuat[3];

      robot->setState(gc, gv);
      robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
      robots.push_back(robot);
    }
  }

  sim->setGravity({0, 0, benchmark::atlas::params.g});
}

double simulationLoop(bool timer = true, bool cntNumContact = true) {
  // resever error vector
  benchmark::atlas::data.setN(unsigned(benchmark::atlas::params.T / benchmark::atlas::params.dt));

  // timer start
  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  Eigen::VectorXd gc(robots[0]->getGeneralizedCoordinateDim());
  Eigen::VectorXd gv(robots[0]->getDOF());
  Eigen::VectorXd tau(robots[0]->getDOF());
  gc.setZero();
  gv.setZero();

  // no gui
  for(int t = 0; t < (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt); t++) {
    sim->integrate1();
    for(int i = 0; i < robots.size(); i++) {
      gc = robots[i]->getGeneralizedCoordinate().e();
      gv = robots[i]->getGeneralizedVelocity().e();
      tau.setZero();
      tau.tail(29) =
          -benchmark::atlas::params.kp.tail(29).cwiseProduct(gc.tail(29))
              - benchmark::atlas::params.kd.cwiseProduct(gv).tail(29);
      robots[i]->setGeneralizedForce(tau);
    }
    sim->integrate2();
    if(cntNumContact) benchmark::atlas::data.numContactList.push_back(sim->getContactProblem()->size());
  }

  end = std::chrono::steady_clock::now();
  return double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
}

int main(int argc, const char* argv[]) {

  benchmark::atlas::addDescToOption(desc);
  benchmark::atlas::getOptionsFromArg(argc, argv, desc);

  benchmark::atlas::getParamsFromYAML(benchmark::atlas::getYamlpath().c_str(),
                                      benchmark::RAI);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::atlas::options.gui << std::endl
                << "Row      : " << benchmark::atlas::options.numRow << std::endl
                << "-----------------------"
  )

  // trial1: get cpu time
  setupSimulation();
  resetWorld();
  double time = simulationLoop(true, false);

  // reset
  robots.clear();
  delete sim;

  // trial2: get contact
  setupSimulation();
  resetWorld();
  simulationLoop(false, true);
  double avgNumContacts = benchmark::atlas::data.computeAvgNumContact();


  // print to screen
  std::cout<<"time taken for "
           << (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt)
           << " steps "<< time <<"s \n";

  if(benchmark::atlas::options.csv)
    benchmark::atlas::printCSV(benchmark::atlas::getCSVpath(),
                               "RAI",
                               "RAI",
                               "RAI",
                               "RAI",
                               benchmark::atlas::options.numRow,
                               avgNumContacts,
                               time);


  delete sim;
  return 0;
}