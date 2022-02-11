//
// Created by kangd on 15.02.18.
//

#include "raisim/World.hpp"

#include "BouncingBenchmark.hpp"

raisim::World* sim;
std::vector<raisim::SingleBodyObject*> objList;
po::options_description desc;

void setupSimulation() {
  sim = new raisim::World();

  // time step
  sim->setTimeStep(benchmark::bouncing::options.dt);

  // erp
  if(benchmark::bouncing::options.erpYN)
    sim->setERP(benchmark::bouncing::params.erp);
  else
    sim->setERP(0);
}

void setupWorld() {
  // materials
  raisim::MaterialManager materials;
  materials.setMaterialPairProp("ground", "ball",
                                benchmark::bouncing::params.mu_ground * benchmark::bouncing::params.mu_ball,
                                benchmark::bouncing::options.e,
                                0);
  sim->updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim->addGround();
  checkerboard->getCollisionObject()->material = "ground";

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m);
      ball->setPosition(i * 2.0, j * 2.0, benchmark::bouncing::params.H);
      ball->getCollisionObject()->material = "ball";
      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});
}

double simulationLoop(bool timer = true, bool error = true) {
  // resever error vector
  benchmark::bouncing::data.setN(unsigned(benchmark::bouncing::params.T / benchmark::bouncing::options.dt));
  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
    // data save
    if (error) {
      double E = 0;
      for(int j = 0; j < objList.size(); j++) {
        E += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
      }
      benchmark::bouncing::data.ballEnergy.push_back(E);
    }

    sim->integrate();
  }

  end = std::chrono::steady_clock::now();
  return double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
}

int main(int argc, const char* argv[]) {

  benchmark::bouncing::addDescToOption(desc);
  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
                                         benchmark::RAI);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::bouncing::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::bouncing::options.csv)
    benchmark::bouncing::printCSV(benchmark::bouncing::getCSVpath(),
                                  "RAI",
                                  "RAI",
                                  "RAI",
                                  "RAI",
                                  time,
                                  error);

   RSINFO(
      std::endl << "CPU time   : " << time << std::endl
                << "mean error : " << error << std::endl
                << "=======================" << std::endl
  )
  
  delete sim;
  return 0;
}