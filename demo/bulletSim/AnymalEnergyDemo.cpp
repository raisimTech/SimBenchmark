//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "BtMbSim.hpp"


//#define VIDEO_SAVE_MODE
#define URDFPATH ROOTPATH "/res/benchmark/ANYmal-energy-benchmark/bullet/robot.urdf"

namespace po = boost::program_options;
std::vector<double> kenergy;

void showplot(double);

int main(int argc, const char* argv[]) {

    std::string urdfPath = std::string(URDFPATH);

    bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);

    auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
    checkerboard->setFrictionCoefficient(0.8);

    auto anymal = sim.addArticulatedSystem(urdfPath, bullet_mb_sim::object::URDF, false);
    anymal->setGeneralizedCoordinate(
            {0, 0, 10,
             1.0, 0.0, 0.0, 0.0,
             0.03, 0.4, -0.8,
             -0.03, 0.4, -0.8,
             0.03, -0.4, 0.8,
             -0.03, -0.4, 0.8});
//  anymal->setGeneralizedCoordinate(
//      {0, 0, 10,
//       1.0, 0.0, 0.0, 0.0,
//       0, 0, 0,
//       0, 0, 0,
//       0, 0, 0,
//       0, 0, 0});
    anymal->setGeneralizedVelocity(
            {0, 0, 0,
             0.2, 1.0, 0.5,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0});
     RSINFO(anymal->getGeneralizedVelocity())

    double g = 0;
    double dt = 0.005;
    sim.setGravity({0, 0, g});
    sim.setTimeStep(dt);

    sim.cameraFollowObject(checkerboard, {15.0, 0.0, 15.0});

    double E0 = anymal->getEnergy({0, 0, g});
    for(int i = 0; i < int(5.0/dt) && sim.visualizerLoop(dt, 1.0); i++) {
        kenergy.push_back(anymal->getEnergy({0, 0, g}));
        sim.integrate();
    }

     RSINFO("initial E = " << E0)
    showplot(E0);
    return 0;
}

void showplot(double E0) {

    int n = kenergy.size();
    Eigen::MatrixXd tdata(n, 1);
    Eigen::MatrixXd Edata(n, 1);
    Eigen::MatrixXd edata(n, 1);

    for(int i = 0; i < n; i++) {
        tdata(i, 0) = i;
        Edata(i, 0) = kenergy[i];
        edata(i, 0) = E0 - kenergy[i];
    }

    rai::Utils::Graph::FigProp2D figure1properties("step", "Energy", "Energy");
    rai::Utils::graph->figure(1, figure1properties);
    rai::Utils::graph->appendData(1,
                                  tdata.data(),
                                  Edata.data(),
                                  n,
                                  "E");
    rai::Utils::graph->drawFigure(1);

    rai::Utils::Graph::FigProp2D figure2properties("step", "Energy error", "Energy error");
    rai::Utils::graph->figure(2, figure2properties);
    rai::Utils::graph->appendData(2,
                                  tdata.data(),
                                  edata.data(),
                                  n,
                                  "error");
    rai::Utils::graph->drawFigure(2);
    rai::Utils::graph->waitForEnter();
}
