//
// Created by kangd on 23.04.18.
//

#include "raisim/World.hpp"

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Multibody/robot.urdf";

  raisim::World_RG sim(800, 600, 0.5, raisim::NO_BACKGROUND);
  sim.setGravity({0, 0, 0});

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1, raisim::GRID);
  auto robot = sim.addArticulatedSystem(urdfPath);
  robot->setGeneralizedCoordinate(
      {0, 0, 1,
       1, 0, 0, 0,
       0, 0, 0});
  robot->setGeneralizedVelocity(
      {0, 0, 0,
       0, 0, 0,
       1, 0, 0});

  sim.cameraFollowObject(checkerboard, {0, 2.5, 1});
  for(int i = 0; i < 10000 && sim.visualizerLoop(0.005, 0.1); i++) {
//    robot->setGeneralizedForce(
//        {0, 0, 0,
//         0, 0, 0,
//         0, 0, 0.1});
    sim.integrate(0.005);
  }

   RSINFO("generalized coordinate = " << robot->getGeneralizedCoordinate());

  return 0;
}
