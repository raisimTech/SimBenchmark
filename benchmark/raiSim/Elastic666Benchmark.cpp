//
// Created by kangd on 15.02.18.
//

#include "raisim/World.hpp"

#include "Elastic666Benchmark.hpp"

raisim::World* sim;
std::vector<raisim::SingleBodyObject*> objList;
po::options_description desc;

void setupSimulation() {
  sim = new raisim::World();

  // erp
  if(benchmark::elasticsixsixsix::options.erpYN)
    sim->setERP(benchmark::elasticsixsixsix::params.erp);
  else
    sim->setERP(0);

  // time step
  sim->setTimeStep(benchmark::elasticsixsixsix::options.dt);
}

double computeEnergy() {
  double energy = 0;
  for(int j = 0; j < objList.size(); j++)
    energy += objList[j]->getEnergy({0, 0, benchmark::elasticsixsixsix::params.g});
  return energy;
};

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::elasticsixsixsix::params.g});

  // materials
  raisim::MaterialManager materials;
  materials.setMaterialPairProp("ground", "ball",
                                0, 1, 0);
  materials.setMaterialPairProp("ball", "ball",
                                0, 1, 0);
  sim->updateMaterialProp(materials);

  // random number generator
  std::mt19937 gen_;
  std::uniform_real_distribution<double> uniDist_(0., 1.);
  gen_.seed(benchmark::elasticsixsixsix::params.randomSeed);

  auto checkerboard = sim->addGround();
  checkerboard->getCollisionObject()->material = "ground";

  for(int i = 0; i < benchmark::elasticsixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::elasticsixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::elasticsixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addSphere(benchmark::elasticsixsixsix::params.ballR,
                                  benchmark::elasticsixsixsix::params.ballM);

        // set position
        double x =
            double(i) * benchmark::elasticsixsixsix::params.gap
                + uniDist_(gen_) * benchmark::elasticsixsixsix::params.perturbation;
        double y =
            double(j) * benchmark::elasticsixsixsix::params.gap
                + uniDist_(gen_) * benchmark::elasticsixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::elasticsixsixsix::params.gap
                + uniDist_(gen_) * benchmark::elasticsixsixsix::params.perturbation
                + benchmark::elasticsixsixsix::params.H;

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
  benchmark::elasticsixsixsix::data.setN(unsigned(benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt));

  // timer start
  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  for(int i = 0; i < (int) (benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt); i++) {
    // data save
    if (error) {
      static double E0 = 0;
      if(i==0)
        E0 = computeEnergy();

      double error = pow(computeEnergy() - E0, 2);
      benchmark::elasticsixsixsix::data.error.push_back(error);
    }

    sim->integrate();
  }

  end = std::chrono::steady_clock::now();
  return double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
}

int main(int argc, const char* argv[]) {

  benchmark::elasticsixsixsix::addDescToOption(desc);
  benchmark::elasticsixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::elasticsixsixsix::getParamsFromYAML(benchmark::elasticsixsixsix::getYamlpath().c_str(),
                                         benchmark::RAI);

   RSINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << "RAI" << std::endl
                << "GUI      : " << benchmark::elasticsixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::elasticsixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::elasticsixsixsix::options.dt << std::endl
                << "Solver   : " << "RAI" << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::elasticsixsixsix::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);


  if(benchmark::elasticsixsixsix::options.csv)
    benchmark::elasticsixsixsix::printCSV(benchmark::elasticsixsixsix::getCSVpath(),
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
