//
// Created by kangd on 11.02.18.
//

#include "OdeSingleBodyObject.hpp"

namespace ode_sim {

object::OdeSingleBodyObject::OdeSingleBodyObject(const dWorldID worldID, const dSpaceID spaceID)
    : worldID_(worldID), spaceID_ (spaceID) {}

object::OdeSingleBodyObject::~OdeSingleBodyObject() {
  if(body_)
    dBodyDestroy(body_);
  if(geometry_)
    dGeomDestroy(geometry_);
}

const benchmark::eQuaternion ode_sim::object::OdeSingleBodyObject::getQuaternion() {
  dQuaternion dquaternion;
  dGeomGetQuaternion(geometry_, dquaternion);
  quatTemp_ = {dquaternion[0], dquaternion[1], dquaternion[2], dquaternion[3]};
  return quatTemp_.e();
}

void ode_sim::object::OdeSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  dQuaternion dquaternion;
  dGeomGetQuaternion(geometry_, dquaternion);
  quat = {dquaternion[0], dquaternion[1], dquaternion[2], dquaternion[3]};
}

const benchmark::eRotationMat ode_sim::object::OdeSingleBodyObject::getRotationMatrix() {
  const dReal* rot = dGeomGetRotation(geometry_);
  rotMatTemp_.e() << rot[0], rot[1], rot[2],
      rot[4], rot[5], rot[6],
      rot[8], rot[9], rot[10];
  return rotMatTemp_.e();
}

void ode_sim::object::OdeSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  const dReal* rot = dGeomGetRotation(geometry_);
  rotation.e() << rot[0], rot[1], rot[2],
      rot[4], rot[5], rot[6],
      rot[8], rot[9], rot[10];
}

const benchmark::eVector3 ode_sim::object::OdeSingleBodyObject::getPosition() {
  const dReal *position = dGeomGetPosition(geometry_);
  posTemp_ = {position[0], position[1], position[2]};
  return posTemp_.e();
}

const benchmark::eVector3 ode_sim::object::OdeSingleBodyObject::getComPosition() {
  const dReal *position = dGeomGetPosition(geometry_);
  posTemp_ = {position[0], position[1], position[2]};
  RSWARN('check if COM = body origin!');
  return posTemp_.e();
}

const benchmark::eVector3 ode_sim::object::OdeSingleBodyObject::getLinearVelocity() {
  const dReal *linearVelocity;
  if(body_) {
    linearVelocity = dBodyGetLinearVel(body_);
    linVelTemp_ = {linearVelocity[0], linearVelocity[1], linearVelocity[2]};
  }
  else {
     RSFATAL('cannot get velocity from static object');
  }
  return linVelTemp_.e();
}

const benchmark::eVector3 ode_sim::object::OdeSingleBodyObject::getAngularVelocity() {
  const dReal *angularVelocity;
  if(body_) {
    angularVelocity = dBodyGetAngularVel(body_);
  }
  else {
     RSFATAL('cannot get velocity from static object');
  }
  angVelTemp_ = {angularVelocity[0], angularVelocity[1], angularVelocity[2]};
  return angVelTemp_.e();
}

void ode_sim::object::OdeSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  const dReal *position = dGeomGetPosition(geometry_);
  pos_w = {position[0], position[1], position[2]};
}

void ode_sim::object::OdeSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  dGeomSetPosition(geometry_, originPosition[0], originPosition[1], originPosition[2]);
}

void ode_sim::object::OdeSingleBodyObject::setPosition(double x, double y, double z) {
  dGeomSetPosition(geometry_, x, y, z);
}

void ode_sim::object::OdeSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  dQuaternion dquaternion = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  dGeomSetQuaternion(geometry_, dquaternion);
}

void ode_sim::object::OdeSingleBodyObject::setOrientation(double w, double x, double y, double z) {
  dQuaternion dquaternion = {w, x, y, z};
  dGeomSetQuaternion(geometry_, dquaternion);
}

void ode_sim::object::OdeSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  dMatrix3 drotation;
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      drotation[4*i + j] = rotationMatrix(i, j);
    }
    drotation[4*i + 3] = 0;
  }
  dGeomSetRotation(geometry_, drotation);
}

void object::OdeSingleBodyObject::setOrientationRandom() {
  Eigen::Vector4d quat(rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform());
  quat /= quat.norm();
  setOrientation(quat(0), quat(1), quat(2), quat(3));
}

void ode_sim::object::OdeSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  setPosition(originPosition);
  setOrientation(quaternion);
}

void ode_sim::object::OdeSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  setPosition(originPosition);
  setOrientation(rotationMatrix);
}

void ode_sim::object::OdeSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity) {
  setVelocity(linearVelocity[0], linearVelocity[1], linearVelocity[2],
              angularVelocity[0], angularVelocity[1], angularVelocity[2]);
}

void ode_sim::object::OdeSingleBodyObject::setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
  if(body_) {
    dBodySetLinearVel(body_, dx, dy, dz);
    dBodySetAngularVel(body_, wx, wy, wz);
  }
  else {
     RSFATAL('cannot set velocity to static object');
  }
}

void object::OdeSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  if(body_) {
    dBodyAddForce(body_, force[0], force[1], force[2]);
  }
  else {
     RSFATAL('cannot set velocity to static object');
  }
}

void object::OdeSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  if(body_) {
    dBodyAddTorque(body_, torque[0], torque[2], torque[2]);
  }
  else {
     RSFATAL('cannot set torque to static object');
  }
}
void object::OdeSingleBodyObject::setRestitutionCoefficient(double restitution) {
  matrialProp_.restitutionCoeff = restitution;
  dGeomSetData(geometry_, &matrialProp_);
}

void object::OdeSingleBodyObject::setFrictionCoefficient(double friction) {
  matrialProp_.frictionalCoeff = friction;
  dGeomSetData(geometry_, &matrialProp_);
}

bool ode_sim::object::OdeSingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

double object::OdeSingleBodyObject::getKineticEnergy() {
  const dReal *angularVelocity;
  const dReal *linearVelocity;
  const dReal* rot = dGeomGetRotation(geometry_);
  if(body_) {
    linearVelocity = dBodyGetLinearVel(body_);
    angularVelocity = dBodyGetAngularVel(body_);
  }
  else {
     RSFATAL('cannot get velocity from static object');
  }

  // cal kinetic energy
  double linEnergy = dDot(linearVelocity, linearVelocity, 3);
  double angEnergy = 0;
  benchmark::Vec<3> angVel = {angularVelocity[0],
                              angularVelocity[1],
                              angularVelocity[2]};

  benchmark::Mat<3,3> I_w;
  benchmark::Mat<3,3> rotation;
  rotation.e() << rot[0], rot[1], rot[2],
      rot[4], rot[5], rot[6],
      rot[8], rot[9], rot[10];
  benchmark::Mat<3,3> I;
  I.e() << mass_.I[0], mass_.I[1], mass_.I[2],
      mass_.I[4], mass_.I[5], mass_.I[6],
      mass_.I[8], mass_.I[9], mass_.I[10];
  benchmark::similarityTransform(rotation, I, I_w);
  benchmark::vecTransposeMatVecMul(angVel, I_w, angEnergy);

  return 0.5 * mass_.mass * linEnergy + 0.5 * angEnergy;
}

double object::OdeSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  const dReal *position = dGeomGetPosition(geometry_);
  double potential = position[0] * gravity[0] +
      position[1] * gravity[1] +
      position[2] * gravity[2];
  return -potential * mass_.mass;
}

double object::OdeSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
}

const benchmark::eVector3 object::OdeSingleBodyObject::getLinearMomentum() {
  const dReal *linearVelocity;
  if(body_) {
    linearVelocity = dBodyGetLinearVel(body_);
    linMomentum_ = {linearVelocity[0] * mass_.mass,
                    linearVelocity[1] * mass_.mass,
                    linearVelocity[2] * mass_.mass};
  }
  else {
     RSFATAL('cannot get velocity from static object');
  }
  return linMomentum_.e();
}

} // ode_sim
