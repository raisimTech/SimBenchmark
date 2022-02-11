//
// Created by kangd on 14.05.18.
//


#include "raisim/World.hpp"
#include "AnymalEnergyBenchmark.hpp"

raisim::World *sim;
std::vector<raisim::SingleBodyObject*> objList;
po::options_description desc;
raisim::ArticulatedSystem* anymal;


void setupSimulation() {
  sim = new raisim::World();

  // set erp 0
  sim->setERP(0);

  // time step
  sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addGround();

  // anymal (internal collision disabled)
  anymal = sim->addArticulatedSystem(benchmark::anymal::freedrop::getURDFpath());
  anymal->setGeneralizedCoordinate({0,
                                    0,
                                    benchmark::anymal::freedrop::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    -0.03, 0.4, -0.8,
                                    0.03, -0.4, 0.8,
                                    -0.03, -0.4, 0.8});
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  // gravity
  sim->setGravity({0, 0, benchmark::anymal::freedrop::params.g});

  // mass and force update
  benchmark::anymal::freedrop::params.M =
      std::accumulate( anymal->getMass().begin(), anymal->getMass().end(), 0.0);

  benchmark::anymal::freedrop::params.F =
      benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 2;
}

double simulationLoop(bool timer = true, bool error = true) {
  // resever error vector
  if(error)
    benchmark::anymal::freedrop::data.setN(
        unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt)
    );

  // timer start
  std::chrono::steady_clock::time_point begin, end;
  begin = std::chrono::steady_clock::now();

  {
    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {
      anymal->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0});
      sim->integrate();
    }
  }

  {
    // step2: freedrop
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt); t++) {
      sim->integrate1();

      anymal->setGeneralizedForce({0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0});

      if(error) {
        if(t==0)
          benchmark::anymal::freedrop::data.E0 = anymal->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});

        benchmark::anymal::freedrop::data.kineticE.push_back(
            anymal->getKineticEnergy()
        );
        benchmark::anymal::freedrop::data.potentialE.push_back(
            anymal->getPotentialEnergy({0, 0, benchmark::anymal::freedrop::params.g})
        );
      }
      sim->integrate2();
    }
  }

  end = std::chrono::steady_clock::now();
  return double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1.0e6;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::RAI);

  RSINFO(
                   "\n=======================" << std::endl
                << "Simulator  : " << "RAI" << std::endl
                << "GUI        : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver     : " << "RAI" << std::endl
                << "Integrator : " << "RAI" << std::endl
                << "Timestep   : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::freedrop::data.computeError();

  // reset
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
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