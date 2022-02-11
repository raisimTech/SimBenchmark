//
// Created by kangd on 09.05.18.
//

#include "raisim/World.hpp"
#include "KaplaBenchmark.hpp"
#include "raisim/RaisimServer.hpp"

raisim::World* sim;
std::vector<raisim::SingleBodyObject*> objList;
po::options_description desc;

void setupSimulation() {
  sim = new raisim::World();

  // erp
  if(benchmark::building::options.erpYN)
    sim->setERP(benchmark::building::params.erp);
  else
    sim->setERP(0);

  // time step
  sim->setTimeStep(benchmark::building::params.dt);

  sim->setContactSolverParam(1.0, 0.7, 1.0,
                             benchmark::building::options.numSolverIter,
                             1e-12);
}

void setupWorld() {
  // add objects
  auto checkerboard = sim->addGround();

  // block size
  const float shortLen = benchmark::building::params.shortLen;
  const float longLen = benchmark::building::params.longLen;
  const float heightLen = benchmark::building::params.heightLen;

  // num of blocks
  // numFloor x numBase + numFloor x (numWall x 2 + 1)
  const int numFloor = benchmark::building::params.numFloor;
  const int numBase = benchmark::building::params.numBase;
  const int numWall = numBase / 2;

  for(int i = 0; i < numFloor; i++) {
    // i floor
    for(int j = 0; j < numBase; j++) {
      // base
      auto base = sim->addBox(shortLen, longLen + 0.05, heightLen, 10.0);
      base->setPosition(j * longLen, 0, i * heightLen * 2 + 0.05);
      objList.push_back(base);
    }

    for(int j = 0; j < numWall; j++) {
      // right wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0);
      wall->setPosition(j * longLen * 2 + 0.1, -0.5 * longLen, i * heightLen * 2 + 0.15);
      objList.push_back(wall);
    }

    for(int j = 0; j < numWall - 1; j++) {
      // left wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0);
      wall->setPosition(j * longLen * 2 + 0.3, 0.5 * longLen, i * heightLen * 2 + 0.15);
      objList.push_back(wall);
    }

    // first wall on left
    auto wall1 = sim->addBox(longLen, shortLen, heightLen, 10.0);
    wall1->setPosition(0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objList.push_back(wall1);

    // last wall on left
    auto wall2 = sim->addBox(longLen, shortLen, heightLen, 10.0);
    wall2->setPosition((numWall - 1) * longLen * 2 + 0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objList.push_back(wall2);
  }

  // gravity
  sim->setGravity({0, 0, benchmark::building::params.g});
}

benchmark::building::Data simulationLoop() {
  // data
  benchmark::building::Data data;
  data.setN(unsigned(benchmark::building::params.T / benchmark::building::params.dt));

  // timer start
  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  int i;
  for(i = 0; i < (int) (benchmark::building::params.T / benchmark::building::params.dt); i++) {
    // num contacts
    data.numContacts.push_back(sim->getContactProblem()->size());

    if(benchmark::building::options.collapse && objList.back()->getPosition()[2] <
        benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
      // break if the building collapses
       RSINFO("building collapsed after " << i << " steps = " << i * benchmark::building::params.dt << " sec!")
      break;
    }

    sim->integrate();
  }

  end = std::chrono::steady_clock::now();
  data.time = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
  data.step = i;
  return data;
}

int main(int argc, const char* argv[]) {

  benchmark::building::addDescToOption(desc);
  benchmark::building::getOptionsFromArg(argc, argv, desc);
  benchmark::building::getParamsFromYAML(benchmark::building::getYamlpath().c_str(),
                                         benchmark::RAI);

  setupSimulation();
  setupWorld();

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator : Raisim" << std::endl
                << "GUI       : " << benchmark::building::options.gui << std::endl
                << "ERP       : " << benchmark::building::options.erpYN << std::endl
                << "Num iter  : " << benchmark::building::options.numSolverIter << std::endl
                << "Tolerance : " << benchmark::building::options.solverTol << std::endl
                << "Timestep  : " << benchmark::building::params.dt << std::endl
                << "Num block : " << objList.size() << std::endl
                << "-----------------------"
  )

  raisim::RaisimServer server(sim);

  if(benchmark::building::options.gui)
    server.launchServer();
  benchmark::building::Data data = simulationLoop();

  if(benchmark::building::options.gui)
    server.killServer();

  if(benchmark::building::options.csv)
    benchmark::building::printCSV(benchmark::building::getCSVpath(),
                                  "RAI",
                                  "RAI",
                                  "RAI",
                                  "RAI",
                                  data.time,
                                  data.step,
                                  data.computeMeanContacts());

   RSINFO(
      std::endl << "Avg. Num Contacts : " << data.computeMeanContacts() << std::endl
                << "CPU time          : " << data.time << std::endl
                << "num steps         : " << data.step << std::endl
                << "speed             : " << data.step / data.time << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}
