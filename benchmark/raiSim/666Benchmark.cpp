//
// Created by kangd on 15.02.18.
//

#include "raisim/World.hpp"
#include "666Benchmark.hpp"
#include "raisim/RaisimServer.hpp"

raisim::World *sim;
std::vector<raisim::SingleBodyObject*> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::sixsixsix::options.gui)
    sim = new raisim::World;
  else
    sim = new raisim::World;

  // erp
  if(benchmark::sixsixsix::options.erpYN)
    sim->setERP(benchmark::sixsixsix::params.erp);
  else
    sim->setERP(0);

  // time step
  sim->setTimeStep(benchmark::sixsixsix::options.dt);
}

double penetrationCheck() {
  double error = 0;
  int numObj = objList.size();

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (objList[i]->getPosition() - objList[j]->getPosition()).norm();

      // error between spheres
      if (dist < benchmark::sixsixsix::params.ballR * 2)
        error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (objList[i]->getPosition()[2] < benchmark::sixsixsix::params.ballR) {
      error +=
          pow(benchmark::sixsixsix::params.ballR - objList[i]->getPosition()[2], 2);
    }
  }

  return error;
}

double computeEnergy() {
  double energy = 0;
  for(int j = 0; j < objList.size(); j++)
    energy += objList[j]->getEnergy({0, 0, benchmark::sixsixsix::params.g});
  return energy;
};

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::sixsixsix::params.g});

  // materials
  raisim::MaterialManager materials;
  materials.setMaterialPairProp("ground", "ball",
                                0, 0, 0.01);
  materials.setMaterialPairProp("ball", "ball",
                                0, 0, 0.01);
  sim->updateMaterialProp(materials);

  // random number generator
  std::mt19937 gen_;
  std::uniform_real_distribution<double> uniDist_(0., 1.);
  gen_.seed(benchmark::sixsixsix::params.randomSeed);


  auto ground = sim->addGround(0, "ground");
  for(int i = 0; i < benchmark::sixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::sixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::sixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addSphere(benchmark::sixsixsix::params.ballR,
                                  benchmark::sixsixsix::params.ballM);

        // set position
        double x =
            double(i) * benchmark::sixsixsix::params.gap
                + uniDist_(gen_) * benchmark::sixsixsix::params.perturbation;
        double y =
            double(j) * benchmark::sixsixsix::params.gap
                + uniDist_(gen_) * benchmark::sixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::sixsixsix::params.gap
                + uniDist_(gen_) * benchmark::sixsixsix::params.perturbation
                + benchmark::sixsixsix::params.H;

        obj->setPosition(x, y, z);
        obj->getCollisionObject()->material = "ball";
//        obj->setOrientationRandom();

        objList.push_back(obj);
      }
    }
  }
}

double simulationLoop(bool timer = true, bool error = true) {
  // resever error vector
  benchmark::sixsixsix::data.setN(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));

  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt); i++) {

    // data save
    if (error) {
      static double E0 = 0;
      if(i==0)
        E0 = computeEnergy();

      double error = penetrationCheck();
      benchmark::sixsixsix::data.error.push_back(error);
    }
//    raisim::MSLEEP(100);
    sim->integrate();
  }

  end = std::chrono::steady_clock::now();
  return double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
}

int main(int argc, const char* argv[]) {

  benchmark::sixsixsix::addDescToOption(desc);
  benchmark::sixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                          benchmark::RAI);

  RSINFO(          "\n=======================" << std::endl
                << "Simulator: " << "RAI" << std::endl
                << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::sixsixsix::options.dt << std::endl
                << "Solver   : " << "RAI" << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  raisim::RaisimServer server(sim);
  if(benchmark::sixsixsix::options.gui)
    server.launchServer();
  simulationLoop(false, true);
  if(benchmark::sixsixsix::options.gui)
    server.killServer();

  double error = benchmark::sixsixsix::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::sixsixsix::options.csv)
    benchmark::sixsixsix::printCSV(benchmark::sixsixsix::getCSVpath(),
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
