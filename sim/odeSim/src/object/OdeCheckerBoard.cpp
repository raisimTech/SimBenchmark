//
// Created by kangd on 12.02.18.
//

#include "OdeCheckerBoard.hpp"
#include "OdeSingleBodyObject.hpp"

ode_sim::object::OdeCheckerBoard::OdeCheckerBoard(dWorldID worldId,
                                            dSpaceID spaceID,
                                            benchmark::CollisionGroupType collisionGroup,
                                            benchmark::CollisionGroupType collisionMask)
    : OdeSingleBodyObject(worldId, spaceID) {

  // geometry
  geometry_ = dCreatePlane(spaceID, 0, 0, 1, 0);
  body_ = 0;

  // material prop
  dGeomSetData(geometry_, &matrialProp_);

  // collision group
  dGeomSetCategoryBits(geometry_, collisionGroup);
  dGeomSetCollideBits(geometry_, collisionMask);
}

ode_sim::object::OdeCheckerBoard::~OdeCheckerBoard() {
}

const Eigen::Map<Eigen::Matrix<double, 4, 1>> ode_sim::object::OdeCheckerBoard::getQuaternion() {
   RSFATAL("CANNOT GET POSE OF STATIC PLANE");
}

void ode_sim::object::OdeCheckerBoard::getQuaternion(benchmark::Vec<4> &quat) {
  quat = {0, 0, 0, 1};
}

const Eigen::Map<Eigen::Matrix<double, 3, 3> > ode_sim::object::OdeCheckerBoard::getRotationMatrix() {
   RSFATAL("CANNOT GET POSE OF STATIC PLANE");
}

void ode_sim::object::OdeCheckerBoard::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
   RSFATAL("CANNOT GET POSE OF STATIC PLANE");
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::OdeCheckerBoard::getPosition() {
   RSFATAL("CANNOT GET POSE OF STATIC PLANE");
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::OdeCheckerBoard::getComPosition() {
   RSFATAL("CANNOT GET POSE OF STATIC PLANE");
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::OdeCheckerBoard::getLinearVelocity() {
   RSFATAL("CANNOT GET VELOCITY OF STATIC PLANE");
}

const Eigen::Map<Eigen::Matrix<double, 3, 1> > ode_sim::object::OdeCheckerBoard::getAngularVelocity() {
   RSFATAL("CANNOT GET VELOCITY OF STATIC PLANE");
}

void ode_sim::object::OdeCheckerBoard::getPosition_W(benchmark::Vec<3> &pos_w) {
  dVector4 planeParams;
  dGeomPlaneGetParams(geometry_, planeParams);
  pos_w = {0, 0, planeParams[3]};
}
void ode_sim::object::OdeCheckerBoard::setPosition(Eigen::Vector3d originPosition) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::OdeCheckerBoard::setPosition(double x, double y, double z) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::OdeCheckerBoard::setOrientation(Eigen::Quaterniond quaternion) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::OdeCheckerBoard::setOrientation(double w, double x, double y, double z) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::OdeCheckerBoard::setOrientation(Eigen::Matrix3d rotationMatrix) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::OdeCheckerBoard::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}
void ode_sim::object::OdeCheckerBoard::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
   RSFATAL("CANNOT SET POSE OF STATIC PLANE");
}

