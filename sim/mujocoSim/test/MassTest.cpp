//
// Created by kangd on 12.05.18.
//

#include "MjcSim.hpp"

int main() {

  // file path
  std::string xmlPath(__FILE__);
  while (xmlPath.back() != '/')
    xmlPath.erase(xmlPath.size() - 1, 1);
  xmlPath += "../../../res/mujoco/test/masstest.xml";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

  // load model from file and check for errors
  mujoco_sim::MjcSim sim(800,
                              600,
                              0.5,
                              xmlPath.c_str(),
                              keyPath.c_str(),
                              benchmark::NO_BACKGROUND);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {10, 0, 5});

  // run simulation for 10 seconds
  sim.setTimeStep(0.01);
  sim.loop(1.0);

   RSINFO(sim.getSingleBodyHandle(1)->getKineticEnergy())
   RSINFO(sim.getSingleBodyHandle(1)->getPotentialEnergy({0, 0, -9.8}))

  return 0;
}
