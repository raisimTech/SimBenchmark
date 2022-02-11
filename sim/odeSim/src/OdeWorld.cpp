//
// Created by kangd on 11.02.18.
//

#include "OdeWorld.hpp"

namespace ode_sim {

// static members
dWorldID ode_sim::OdeWorld::dynamicsWorld_;
dJointGroupID ode_sim::OdeWorld::contactGroup_;
std::vector<Single3DContactProblem> ode_sim::OdeWorld::contactProblemList_;

void message(int errnum, const char *msg, va_list ap) {
  // no debug message
}

ode_sim::OdeWorld::OdeWorld(SolverOption solverOption) : solverOption_(solverOption) {

  // world
  dInitODE();
  dynamicsWorld_ = dWorldCreate();
  dSetMessageHandler(0);

  dVector3 Center = {0, 0, 0, 0};
  dVector3 Extents = {10, 0, 10, 0};
  space_ = dQuadTreeSpaceCreate(0, Center, Extents, 7);
//  space_ = dHashSpaceCreate(0);
  contactGroup_ = dJointGroupCreate(0);

  dWorldSetGravity(dynamicsWorld_, gravity_[0], gravity_[1], gravity_[2]);

  ///  auto disable
  dWorldSetAutoDisableLinearThreshold(dynamicsWorld_, 0);
  dWorldSetAutoDisableAngularThreshold(dynamicsWorld_, 0);
//  dWorldSetAutoDisableAverageSamplesCount(dynamicsWorld_, 10);
//  dWorldSetAutoDisableFlag(dynamicsWorld_, 1);

  ///  parameters
  dWorldSetLinearDamping(dynamicsWorld_, 0);
  dWorldSetAngularDamping(dynamicsWorld_, 0);
  dWorldSetCFM(dynamicsWorld_, 0);
  dWorldSetERP(dynamicsWorld_, 0);
//  dWorldSetMaxAngularSpeed(dynamicsWorld_, 200);
//  dWorldSetContactMaxCorrectingVel(dynamicsWorld_,0.1);
//  dWorldSetContactSurfaceLayer(dynamicsWorld_,0.001);

  // num iteration of quick solver. (default value is 20.)
//  dWorldSetQuickStepNumIterations(dynamicsWorld_, 50);

  // no debug message
  dSetMessageHandler(message);
}

ode_sim::OdeWorld::~OdeWorld() {

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

  // remove world
  dJointGroupDestroy(contactGroup_);
  dSpaceDestroy(space_);
  dWorldDestroy(dynamicsWorld_);
  dCloseODE();
}

// collision detecting callback
void ode_sim::OdeWorld::nearCallback(void *data, dGeomID o1, dGeomID o2) {
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
    return;

  dContact contact[maxContactsPerBody];   // up to MAX_CONTACTS contacts per box-box

  for (i=0; i<maxContactsPerBody; i++) {
    object::MetrialProp *prop1 = (object::MetrialProp*)dGeomGetData(o1);
    object::MetrialProp *prop2 = (object::MetrialProp*)dGeomGetData(o2);

    contact[i].surface.mode = dContactBounce | dContactApprox1;

    if(prop1 && prop2) {
      contact[i].surface.mu = prop1->frictionalCoeff * prop2->frictionalCoeff;
      contact[i].surface.bounce = prop1->restitutionCoeff * prop2->restitutionCoeff;
    } else {
      contact[i].surface.mu = 0.8;
      contact[i].surface.bounce = 0;
    }
  }

  if (int numc = dCollide(o1,o2, maxContactsPerBody, &contact[0].geom,
                          sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (dynamicsWorld_, contactGroup_, contact+i);
      dJointAttach (c,b1,b2);

      contactProblemList_.emplace_back(contact[i].geom.pos, contact[i].geom.normal);
    }
  }
}


ode_sim::object::OdeSphere *ode_sim::OdeWorld::addSphere(double radius,
                                                         double mass,
                                                         benchmark::CollisionGroupType collisionGroup,
                                                         benchmark::CollisionGroupType collisionMask) {
  auto *sphere = new ode_sim::object::OdeSphere(radius, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(sphere);
  return sphere;
}

ode_sim::object::OdeBox *ode_sim::OdeWorld::addBox(double xLength,
                                                   double yLength,
                                                   double zLength,
                                                   double mass,
                                                   benchmark::CollisionGroupType collisionGroup,
                                                   benchmark::CollisionGroupType collisionMask) {
  auto *box = new ode_sim::object::OdeBox(xLength, yLength, zLength, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(box);
  return box;
}

ode_sim::object::OdeCapsule *ode_sim::OdeWorld::addCapsule(double radius,
                                                           double height,
                                                           double mass,
                                                           benchmark::CollisionGroupType collisionGroup,
                                                           benchmark::CollisionGroupType collisionMask) {
  auto *capsule = new ode_sim::object::OdeCapsule(radius, height, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(capsule);
  return capsule;
}

ode_sim::object::OdeCylinder *ode_sim::OdeWorld::addCylinder(double radius,
                                                             double height,
                                                             double mass,
                                                             benchmark::CollisionGroupType collisionGroup,
                                                             benchmark::CollisionGroupType collisionMask) {
  auto *cylinder = new ode_sim::object::OdeCylinder(radius, height, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(cylinder);
  return cylinder;
}

ode_sim::object::OdeCheckerBoard *ode_sim::OdeWorld::addCheckerboard(double gridSize,
                                                                     double xLength,
                                                                     double yLength,
                                                                     double reflectanceI,
                                                                     bo::CheckerboardShape shape,
                                                                     benchmark::CollisionGroupType collisionGroup,
                                                                     benchmark::CollisionGroupType collisionMask) {
  auto *checkerBoard = new ode_sim::object::OdeCheckerBoard(dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

ode_sim::object::OdeArticulatedSystem *ode_sim::OdeWorld::addArticulatedSystem(std::string urdfPath,
                                                                               benchmark::CollisionGroupType collisionGroup,
                                                                               benchmark::CollisionGroupType collisionMask) {
  object::OdeArticulatedSystem *articulatedSystem =
      new object::OdeArticulatedSystem(urdfPath, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(articulatedSystem);
  return articulatedSystem;
}

void ode_sim::OdeWorld::setGravity(const benchmark::Vec<3> &gravity) {
  dVector3 dgravity = {gravity[0], gravity[1], gravity[2]};
  memcpy(gravity_, dgravity, sizeof(dVector3));
  dWorldSetGravity(dynamicsWorld_, gravity_[0], gravity_[1], gravity_[2]);
}

void ode_sim::OdeWorld::setERP(double erp) {
  dWorldSetERP(dynamicsWorld_, erp);
}

void ode_sim::OdeWorld::integrate(double dt) {

  // collision detection
  contactProblemList_.clear();
  dSpaceCollide(space_, 0, &nearCallback);

  // collision solving
  if(solverOption_ == SOLVER_QUICK)
  {
    dWorldQuickStep(dynamicsWorld_, dt);
  }
  else
  {
    dWorldStep(dynamicsWorld_, dt);
  }

  dJointGroupEmpty(contactGroup_);
}

int ode_sim::OdeWorld::getNumObject() {
  return objectList_.size();
}

const std::vector<ode_sim::Single3DContactProblem> *ode_sim::OdeWorld::getCollisionProblem() {
  return &contactProblemList_;
}

void OdeWorld::integrate1(double dt) {
   RSFATAL("not supported for ode")
}

void OdeWorld::integrate2(double dt) {
   RSFATAL("not supported for ode")
}


} // ode_sim

