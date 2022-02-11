//
// Created by kangd on 09.05.18.
//

#include <DartSim.hpp>

#include "KaplaBenchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::building::options.gui)
    sim = new dart_sim::DartSim(800, 600, 0.05,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::dart::options.solverOption);
  else
    sim = new dart_sim::DartSim(benchmark::dart::options.solverOption);

  // time step
  sim->setTimeStep(benchmark::building::params.dt);

  /// no erp for dart
  if(benchmark::building::options.erpYN)
     RSFATAL("erp is not supported for dart")

  // increases max contact numbers
  sim->setMaxContacts(5000);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(10.0, 400.0, 400.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

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

      if(benchmark::building::options.gui)
        base.visual()[0]->setColor({0.0, 0.0, 1.0});
    }

    for(int j = 0; j < numWall; j++) {
      // right wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0);
      wall->setPosition(j * longLen * 2 + 0.1, -0.5 * longLen, i * heightLen * 2 + 0.15);
      objList.push_back(wall);

      if(benchmark::building::options.gui)
        wall.visual()[0]->setColor({0.0, 1.0, 0.0});
    }

    for(int j = 0; j < numWall - 1; j++) {
      // left wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0);
      wall->setPosition(j * longLen * 2 + 0.3, 0.5 * longLen, i * heightLen * 2 + 0.15);
      objList.push_back(wall);

      if(benchmark::building::options.gui)
        wall.visual()[0]->setColor({1.0, 0.0, 0.0});
    }

    // first wall on left
    auto wall1 = sim->addBox(longLen, shortLen, heightLen, 10.0);
    wall1->setPosition(0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objList.push_back(wall1);

    // last wall on left
    auto wall2 = sim->addBox(longLen, shortLen, heightLen, 10.0);
    wall2->setPosition((numWall - 1) * longLen * 2 + 0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objList.push_back(wall2);

    if(benchmark::building::options.gui) {
      wall1.visual()[0]->setColor({1.0, 0.0, 0.0});
      wall2.visual()[0]->setColor({1.0, 0.0, 0.0});
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::building::params.g});

  if(benchmark::building::options.gui) {
    sim->setLightPosition((float)benchmark::building::params.lightPosition[0],
                          (float)benchmark::building::params.lightPosition[1],
                          (float)benchmark::building::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {0, 5, 2});
  }
}

benchmark::building::Data simulationLoop() {
  if(benchmark::building::options.saveVideo)
    sim->startRecordingVideo("/tmp", "rai-building");

  // data
  benchmark::building::Data data;
  data.setN(unsigned(benchmark::building::params.T / benchmark::building::params.dt));

  // timer start
  StopWatch watch;
  watch.start();

  int i;
  for(i = 0; i < (int) (benchmark::building::params.T / benchmark::building::params.dt); i++) {
    // gui
    if (benchmark::building::options.gui && !sim->visualizerLoop(benchmark::building::params.dt))
      break;

    // num contacts
    data.numContacts.push_back(sim->getWorldNumContacts());

    if(benchmark::building::options.collapse && objList.back()->getPosition()[2] <
        benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
      // break if the building collapses
       RSINFO("building collapsed after " << i << " steps = " << i * benchmark::building::params.dt << " sec!")
      break;
    }

    sim->integrate();
  }

  if(benchmark::building::options.saveVideo)
    sim->stopRecordingVideo();

  data.time = watch.measure();
  data.step = i;
  return data;
}

int main(int argc, const char* argv[]) {

  benchmark::building::addDescToOption(desc);
  benchmark::dart::addDescToOption(desc);

  benchmark::building::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::building::getParamsFromYAML(benchmark::building::getYamlpath().c_str(),
                                         benchmark::DART);

  setupSimulation();
  setupWorld();

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::building::options.gui << std::endl
                << "ERP      : " << benchmark::building::options.erpYN << std::endl
                << "Timestep : " << benchmark::building::params.dt << std::endl
                << "Num block: " << objList.size() << std::endl
                << "-----------------------"
  )

  benchmark::building::Data data = simulationLoop();

  if(benchmark::building::options.csv)
    benchmark::building::printCSV(benchmark::building::getCSVpath(),
                                  benchmark::dart::options.simName,
                                  benchmark::dart::options.solverName,
                                  benchmark::dart::options.detectorName,
                                  benchmark::dart::options.integratorName,
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
