//
// Created by kangd on 16.03.18.
//

#include "DartSim.hpp"

namespace dart_sim {

DartSim::DartSim(int windowWidth,
                           int windowHeight,
                           float cms,
                           int flags,
                           SolverOption solverOption,
                           CollisionDetectorOption detectorOption) :
    world_(solverOption, detectorOption),
    benchmark::WorldRG(windowWidth, windowHeight, cms, flags) {}

DartSim::DartSim(SolverOption solverOption, CollisionDetectorOption detectorOption) :
    world_(solverOption, detectorOption),
    benchmark::WorldRG() {}

benchmark::SingleBodyHandle DartSim::addSphere(double radius,
                                                double mass,
                                                benchmark::CollisionGroupType collisionGroup,
                                                benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addSphere(radius, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle DartSim::addBox(double xLength,
                                             double yLength,
                                             double zLength,
                                             double mass,
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

DartSim::~DartSim() {
  if(!isEnded_ && isReady_)
    visEnd();
}

benchmark::SingleBodyHandle DartSim::addCapsule(double radius,
                                                 double height,
                                                 double mass,
                                                 benchmark::CollisionGroupType collisionGroup,
                                                 benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCapsule(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Capsule(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle DartSim::addCylinder(double radius,
                                                  double height,
                                                  double mass,
                                                  benchmark::CollisionGroupType collisionGroup,
                                                  benchmark::CollisionGroupType collisionMask) {
  benchmark::SingleBodyHandle handle(world_.addCylinder(radius, height, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Cylinder(radius, height, true));
  processSingleBody(handle);
  return handle;
}

benchmark::SingleBodyHandle DartSim::addCheckerboard(double gridSize,
                                                      double xLength,
                                                      double yLength,
                                                      double reflectanceI,
                                                      bo::CheckerboardShape shape,
                                                      benchmark::CollisionGroupType collisionGroup,
                                                      benchmark::CollisionGroupType collisionMask,
                                                      int flags) {
  benchmark::SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, shape, collisionGroup, collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & bo::GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

ArticulatedSystemHandle DartSim::addArticulatedSystem(std::string nm,
                                                       benchmark::CollisionGroupType collisionGroup,
                                                       benchmark::CollisionGroupType collisionMask) {
  ArticulatedSystemHandle handle(
      world_.addArticulatedSystem(nm, collisionGroup, collisionMask), {}, {});
  if(!gui_) {
    asHandles_.push_back(handle);
    return handle;
  }

  // for mesh file path
  while (nm.back() != '/')
    nm.erase(nm.size() - 1, 1);

  for (int i = 0; i < handle->visObj.size(); i++) {
    switch (std::get<3>(handle->visObj[i])) {
      case benchmark::object::Shape::Box:
        handle.visual().push_back(new rai_graphics::object::Box(handle->visProps_[i].second.v[0],
                                                                handle->visProps_[i].second.v[1],
                                                                handle->visProps_[i].second.v[2], true));
        break;
      case benchmark::object::Shape::Cylinder:
        handle.visual().push_back(new rai_graphics::object::Cylinder(handle->visProps_[i].second.v[0],
                                                                     handle->visProps_[i].second.v[1], true));
        break;
      case benchmark::object::Shape::Sphere:
        handle.visual().push_back(new rai_graphics::object::Sphere(handle->visProps_[i].second.v[0], true));
        break;
      case benchmark::object::Shape::Mesh:
        checkFileExistance(handle->visProps_[i].first);
        handle.visual().push_back(new rai_graphics::object::Mesh(handle->visProps_[i].first,
                                                                 handle->visProps_[i].second.v[0]));
        break;
    }
    handle.visual().back()->setColor({float(std::get<4>(handle->visObj[i]).v[0]),
                                      float(std::get<4>(handle->visObj[i]).v[1]),
                                      float(std::get<4>(handle->visObj[i]).v[2])});
    processGraphicalObject(handle.visual().back(), std::get<2>(handle->visObj[i]));
  }

  for (int i = 0; i < handle->visColObj.size(); i++) {
    switch (std::get<3>(handle->visColObj[i])) {
      case benchmark::object::Shape::Box:
        handle.alternateVisual().push_back(new rai_graphics::object::Box(handle->visColProps_[i].second.v[0],
                                                                         handle->visColProps_[i].second.v[1],
                                                                         handle->visColProps_[i].second.v[2],
                                                                         true));
        break;
      case benchmark::object::Shape::Cylinder:
        handle.alternateVisual().push_back(new rai_graphics::object::Cylinder(handle->visColProps_[i].second.v[0],
                                                                              handle->visColProps_[i].second.v[1],
                                                                              true));
        break;
      case benchmark::object::Shape::Sphere:
        handle.alternateVisual().push_back(new rai_graphics::object::Sphere(handle->visColProps_[i].second.v[0],
                                                                            true));
        break;
      case benchmark::object::Shape::Mesh:
       RSFATAL("mesh collision body is not supported yet");
        break;
      default:  RSFATAL("unsupported type: ");
        break;
    }
    processGraphicalObject(handle.alternateVisual().back(), std::get<2>(handle->visColObj[i]));
  }

  asHandles_.push_back(handle);
  return handle;
}

void DartSim::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity.x(), gravity.y(), gravity.z()});
}

void DartSim::setTimeStep(double timeStep) {
  timeStep_ = timeStep;
  world_.setTimeStep(timeStep);
}

void DartSim::integrate() {
  world_.integrate();
}

void DartSim::loop(double realTimeFactor) {
  while (visualizerLoop(timeStep_, realTimeFactor))
    integrate();
}

void DartSim::loop(double dt, double realTimeFactor) {
   RSFATAL("use setTimeStep(double dt) + loop(double realTimeFactor) instead")
}

void DartSim::integrate(double dt) {
   RSFATAL("use setTimeStep(double dt) + integrate() instead")
}

void DartSim::setERP(double erp, double erp2, double frictionErp) {

}

int DartSim::getNumObject() {
  return world_.getNumObject();
}

void DartSim::updateFrame() {
   RSFATAL_IF(!gui_, "use different constructor for visualization")
  const bool showAlternateGraphicsIfexists = gui_->getCustomToggleState(3);

  for (auto &as : asHandles_) {
    benchmark::Vec<4> quat;
    benchmark::Vec<3> pos;
    benchmark::Vec<4> color;

    // update visuals for articulated system
    as->updateVisuals();

    if (showAlternateGraphicsIfexists) {
      /// update collision objects
      for (int i = 0; i < as->getVisColOb().size(); i++) {
        as.alternateVisual()[i]->setVisibility(true);
        pos = std::get<1>(as->getVisColOb()[i]);
        as.alternateVisual()[i]->setPos(
            pos[0],
            pos[1],
            pos[2]);
        rotMatToQuat(std::get<0>(as->getVisColOb()[i]), quat);
        as.alternateVisual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(as.alternateVisual()[i], as.hidable);
      }

      for (int i = 0; i < as->getVisOb().size(); i++)
        as.visual()[i]->setVisibility(false);

    } else {
      for (int i = 0; i < as->getVisOb().size(); i++) {
        as.visual()[i]->setVisibility(true);
        if (!as.visual()[i]->isVisible()) continue;
        pos = std::get<1>(as->getVisOb()[i]);
        color = std::get<4>(as->getVisOb()[i]);
        as.visual()[i]->setPos(
            pos[0],
            pos[1],
            pos[2]
        );
        rotMatToQuat(std::get<0>(as->getVisOb()[i]), quat);
        as.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        as.visual()[i]->setColor({float(color[0]),
                                  float(color[1]),
                                  float(color[2])});
        as.visual()[i]->setTransparency(float(color[3]));
        adjustTransparency(as.visual()[i], as.hidable);
      }
      for (int i = 0; i < as->getVisColOb().size(); i++)
        as.alternateVisual()[i]->setVisibility(false);
    }
  }

  benchmark::Vec<3> bodyPosition;
  benchmark::Vec<4> quat;

  for (auto sb : sbHandles_) {
    sb->getPosition_W(bodyPosition);
    sb->getQuaternion(quat);

    if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
      for (auto *go: sb.alternateVisual()) go->setVisibility(false);
      for (auto *go: sb.visual()) {
        go->setVisibility(true);
        go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
        go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(go, sb.hidable);
      }
    } else {
      for (auto *go: sb.visual()) go->setVisibility(false);
      for (auto *go: sb.visual()) {
        go->setVisibility(true);
        go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
        go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(go, sb.hidable);
      }
    }
  }

//  raisim::Mat<3, 3> rot;
//  for (auto sb : comHandles_) {
//    sb->getPosition_W(bodyPosition);
//    sb->getRotationMatrix(rot);
//    auto &trans = sb->getTransformation();
//
//    if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
//      for (auto *go: sb.alternateVisual()) go->setVisibility(false);
//      for (int i = 0; i < sb.visual().size(); i++) {
//        Vec<3> pos;
//        Vec<4> quat;
//        matvecmul(rot, trans[i].pos, pos);
//        sb.visual()[i]->setPos(bodyPosition.v[0] + pos.v[0],
//                               bodyPosition.v[1] + pos.v[1],
//                               bodyPosition.v[2] + pos.v[2]);
//        matmul(rot, trans[i].rot, rot);
//        rotMatToQuat(rot, quat);
//        sb.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
//        adjustTransparency(sb.visual()[i], sb.hidable);
//      }
//    }
//  }

  /// contact points
  if (gui_->getCustomToggleState(1)) {
    contactPointMarker_->mutexLock();
    contactPointMarker_->clearGhost();
    for (auto &pro: world_.getCollisionProblem()) {
      Eigen::Vector3d pos;
      pos[0] = pro.pos_[0];
      pos[1] = pro.pos_[1];
      pos[2] = pro.pos_[2];
      contactPointMarker_->addGhost(pos);
    }
    contactPointMarker_->mutexUnLock();
  } else
    contactPointMarker_->clearGhost();

  /// contact forces
//  if (gui_->getCustomToggleState(2)) {
//    double maxForce = 0;
//    for (auto &pro: *world_.getCollisionProblem())
//      maxForce = (maxForce < pro.imp_i.norm()) ? pro.imp_i.norm() : maxForce;
//    contactNormalArrow_->mutexLock();
//    contactNormalArrow_->clearGhost();
//    for (auto &pro: world_.getObjList()) {
//      for (auto &con: pro->getPerObjectContact().getContacts()) {
//        const double norm = con.getImpulse()->norm();
//        Eigen::Vector3d pos(con.getPosition().v[0], con.getPosition().v[1], con.getPosition().v[2]);
//        Vec<3> scaledImp;
//        matTransposevecmul(con.getContactFrame(), *con.getImpulse(), scaledImp);
//        vecScalarMul(1.0 / norm, scaledImp);
//        Eigen::Vector3d dir = scaledImp.e();
//        Eigen::Vector3f color(norm / maxForce, 0.2, 1 - norm / maxForce);
//        Eigen::Vector3f scale(norm / maxForce, 1, 1);
//        contactNormalArrow_->addGhostWithVector(pos, dir, color, scale);
//      }
//    }
//    contactNormalArrow_->mutexUnLock();
//  } else
//    contactNormalArrow_->clearGhost();

  /// frames and COM
  if (gui_->getCustomToggleState(4)) {

    frameX_->mutexLock();
    frameY_->mutexLock();
    frameZ_->mutexLock();
    graphicalComMarker_->mutexLock();

    frameX_->clearGhost();
    frameY_->clearGhost();
    frameZ_->clearGhost();
    graphicalComMarker_->clearGhost();
    Eigen::Vector3f colorR(1, 0, 0), colorG(0, 1, 0), colorB(0, 0, 1);
    Eigen::Vector3d xdir, ydir, zdir;

    for (auto *cf: framesAndCOMobj_) {
      if (!cf->isVisualizeFramesAndCom()) continue;
      Eigen::Vector3d pos = cf->getPosition();
      Eigen::Matrix3d dir = cf->getRotationMatrix();
      Eigen::Vector3f scale(1, 1, 1);

      xdir = dir.col(0);
      ydir = dir.col(1);
      zdir = dir.col(2);

      frameX_->addGhostWithVector(pos, xdir, colorR, scale);
      frameY_->addGhostWithVector(pos, ydir, colorG, scale);
      frameZ_->addGhostWithVector(pos, zdir, colorB, scale);
      graphicalComMarker_->addGhost(pos);
    }
    frameX_->mutexUnLock();
    frameY_->mutexUnLock();
    frameZ_->mutexUnLock();
    graphicalComMarker_->mutexUnLock();
  } else {
    frameX_->clearGhost();
    frameY_->clearGhost();
    frameZ_->clearGhost();
    graphicalComMarker_->clearGhost();
  }

  if(visualizerFlags_ & benchmark::DISABLE_INTERACTION)
    return;

  /// interaction
//  if (gui_->isInteracting()) {
//    auto indices = interactionIdx_.find(gui_->getInteractingObjectID());
//    interactionForce_ = gui_->getInteractionMagnitude();
//    interactionForce_ *= interactionForce_.norm() * 20.0 * world_.getObjList()[indices->second.first]->getMass(indices->second.second);
//    objToInteract_ = world_.getObjList()[indices->second.first];
//    objToInteractLocalIdx_ = indices->second.second;
//    std::stringstream inStr;
//    inStr << std::setprecision(3) << interactionForce_.norm() << "N";
//    gui_->changeMenuText(1, false, inStr.str());
//    gui_->setMenuPositionNextToCursor(1);
//  } else {
//    gui_->changeMenuText(1, false, "");
//    objToInteract_ = nullptr;
//  }

  /// deletion
//  if (gui_->getKeyboardEvent(rai_graphics::KeyboardEvent::DEL)) {
//    auto indices = interactionIdx_.find(gui_->getInteractingObjectID());
//    if (indices != interactionIdx_.end()) {
//      object::Object *obj = world_.getObjList()[indices->second.first];
//      long int id =
//          std::find_if(sbHandles_.begin(), sbHandles_.end(), [obj](const SingleBodyHandle &a) { return a.s_ == obj; })
//              - sbHandles_.begin();
//      if (id == sbHandles_.size()) {
//        long int id = std::find_if(asHandles_.begin(),
//                                   asHandles_.end(),
//                                   [obj](const ArticulatedSystemHandle &a) { return a.s_ == obj; })
//            - asHandles_.begin();
//        if (!id == asHandles_.size())
//          removeObject(asHandles_[id]);
//      } else {
//        removeObject(sbHandles_[id]);
//      }
//      interactionIdx_.erase(gui_->getInteractingObjectID());
//    }
//  }
}
void DartSim::setMaxContacts(int maxcontacts) {
  world_.setMaxContacts(maxcontacts);
}
int DartSim::getWorldNumContacts() {
  return (int) world_.getCollisionProblem().size();
}

void DartSim::integrate1(double dt) {
   RSFATAL("not supported for dart")
}

void DartSim::integrate2(double dt) {
   RSFATAL("not supported for dart")
}

} // dart_sim