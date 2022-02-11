//
// Created by kangd on 15.04.18.
//

#include "OdeArticulatedSystem.hpp"

namespace ode_sim {
namespace object {

OdeArticulatedSystem::OdeArticulatedSystem(std::string urdfFile,
                                           const dWorldID worldID,
                                           const dSpaceID spaceID,
                                           int collisionGroup,
                                           int collisionMask)
    : worldID_(worldID), spaceID_(spaceID), collisionGroup_(collisionGroup), collisionMask_(collisionMask) {

  std::ifstream model_file(urdfFile);
   RSFATAL_IF(!model_file.good(), "Error opening file: " << urdfFile);

  // reserve memory for the contents of the file
  std::string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  unsigned long searchPosition = 0;
  jointsNames_ = std::vector<std::string>();

  // find all joint names
  while (true) {
    unsigned long position = model_xml_string.find("joint name", searchPosition);
    if (position == std::string::npos) break;
    unsigned long nameStart = model_xml_string.find("\"", position);
    unsigned long nameEnd = model_xml_string.find("\"", nameStart + 1);

    if (std::find(jointsNames_.begin(), jointsNames_.end(), model_xml_string.substr(nameStart + 1, nameEnd - nameStart - 1)) == jointsNames_.end())
      jointsNames_.push_back(model_xml_string.substr(nameStart + 1, nameEnd - nameStart - 1));
    searchPosition = nameEnd;
  }

  boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDFFile(urdfFile);
  boost::shared_ptr<const urdf::Link> rootLink = urdf_model->getRoot();

  if (rootLink->name != "world") {
    // float base
    isFixed_ = false;
    dof_ = 6;
    stateDimension_ = 7;
  } else {
    // fixed base
    isFixed_ = true;
    dof_ = 0;
    stateDimension_ = 0;

    if (rootLink->child_links[0]->parent_joint->type == urdf::Joint::FIXED)
      rootLink = rootLink->child_links[0];
  }

  // rootlink's parent is world
  rootLink_.parentIdx_ = -1;

  // process links recursively
  processLinkFromUrdf(rootLink, rootLink_, jointsNames_);

  // init articulated system
  init();
}

void OdeArticulatedSystem::processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink,
                                               Link &raiLink,
                                               std::vector<std::string> &jointsOrder) {

  // link init
  raiLink.name_ = urdfLink->name;
  raiLink.odeBody_ = dBodyCreate(worldID_);

  /// parent
  if(urdfLink->getParent()) {
    raiLink.parentJointName_ = urdfLink->parent_joint->name;
    raiLink.parentName_ = urdfLink->getParent()->name;
  } else {
    raiLink.parentJointName_ = "";
    raiLink.parentName_ = "";
  }

  /// inertial
  if (urdfLink->inertial) {
    raiLink.inertial_.mass_ = urdfLink->inertial->mass;
    raiLink.inertial_.inertia_.e() << urdfLink->inertial->ixx, urdfLink->inertial->ixy, urdfLink->inertial->ixz,
        urdfLink->inertial->ixy, urdfLink->inertial->iyy, urdfLink->inertial->iyz,
        urdfLink->inertial->ixz, urdfLink->inertial->ixy, urdfLink->inertial->izz;

    raiLink.inertial_.pos_.e() << urdfLink->inertial->origin.position.x,
        urdfLink->inertial->origin.position.y,
        urdfLink->inertial->origin.position.z;

    benchmark::Vec<3> rpy;
    urdfLink->inertial->origin.rotation.getRPY(rpy[0], rpy[1], rpy[2]);
    benchmark::rpyToRotMat_intrinsic(rpy, raiLink.inertial_.rotmat_);
  }
  else {
    raiLink.inertial_.mass_ = 0;
    raiLink.inertial_.inertia_.setZero();
    raiLink.inertial_.pos_.setZero();
    raiLink.inertial_.rotmat_.setIdentity();
  }

  /// collision
  if (urdfLink->collision) {
    // collision object
    for (auto &col: urdfLink->collision_array) {
      if (col->geometry->type == urdf::Geometry::BOX) {
        // box
        auto box = boost::dynamic_pointer_cast<urdf::Box>(col->geometry);
        raiLink.collision_.colShape_.push_back(bo::Shape::Box);
        raiLink.collision_.colShapeParam_.push_back({box->dim.x, box->dim.y, box->dim.z, 0});
      }
      else if (col->geometry->type == urdf::Geometry::CYLINDER) {
        // cylinder
        auto cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(col->geometry);
        raiLink.collision_.colShape_.push_back(bo::Shape::Cylinder);
        raiLink.collision_.colShapeParam_.push_back({cyl->radius, cyl->length, 0, 0});
      }
      else if (col->geometry->type == urdf::Geometry::SPHERE) {
        // sphere
        auto sph = boost::dynamic_pointer_cast<urdf::Sphere>(col->geometry);
        raiLink.collision_.colShape_.push_back(bo::Shape::Sphere);
        raiLink.collision_.colShapeParam_.push_back({sph->radius, 0, 0, 0});
      }
      else {
         RSFATAL("mesh collision body is not supported yet");
      }

      // relative position w.r.t. parent joint
      auto &pos = col->origin.position;
      double r, p, y;
      col->origin.rotation.getRPY(r, p, y);
      raiLink.collision_.colObjOrigin_.push_back({pos.x, pos.y, pos.z});
      raiLink.collision_.colOrientation({r, p, y});
    }
  }

  /// visual
  if (urdfLink->visual) {
    for (auto &vis: urdfLink->visual_array) {
      if (vis->geometry->type == urdf::Geometry::BOX) {
        // box
        auto box = boost::dynamic_pointer_cast<urdf::Box>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Box);
        raiLink.visual_.visShapeParam_.push_back({box->dim.x, box->dim.y, box->dim.z, 0});
        raiLink.visual_.meshFileNames_.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::CYLINDER) {
        // cylinder
        auto cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Cylinder);
        raiLink.visual_.visShapeParam_.push_back({cyl->radius, cyl->length, 0, 0});
        raiLink.visual_.meshFileNames_.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::SPHERE) {
        // sphere
        auto sph = boost::dynamic_pointer_cast<urdf::Sphere>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Sphere);
        raiLink.visual_.visShapeParam_.push_back({sph->radius, 0, 0, 0});
        raiLink.visual_.meshFileNames_.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::MESH) {
        // mesh
        auto mes = boost::dynamic_pointer_cast<urdf::Mesh>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Mesh);
        raiLink.visual_.visShapeParam_.push_back({mes->scale.x, mes->scale.y, mes->scale.z, 0});
        raiLink.visual_.meshFileNames_.push_back(mes->filename);
      }

      if (vis->material)
        raiLink.visual_.visColor_.push_back(
            {vis->material->color.r,
             vis->material->color.g,
             vis->material->color.b,
             vis->material->color.a});
      else
        raiLink.visual_.visColor_.push_back(
            {0.7, 0.7, 0.7, 1.0});

      auto &pos = vis->origin.position;
      double r, p, y;
      vis->origin.rotation.getRPY(r, p, y);
      raiLink.visual_.visObjOrigin_.push_back({pos.x, pos.y, pos.z});
      raiLink.visual_.visOrientation({r, p, y});
    }
  }

  /// urdfreader order joints alphabetically. We follow the order defined in the urdf
  std::vector<int> childOrder;

  for (auto &name : jointsOrder)
    for (int i = 0; i < urdfLink->child_links.size(); i++)
      if (urdfLink->child_links[i]->parent_joint->name == name)
        childOrder.push_back(i);

  int jointsNumber = childOrder.size();

  /// children links
  for(int i = 0; i < urdfLink->child_links.size(); i++) {
    Link *childRef;
    auto ch = urdfLink->child_links[childOrder[i]];
    auto &jnt = ch->parent_joint;
//    childRef->parentJoint_.jointName_ = jnt->name;

    switch (jnt->type) {
      case urdf::Joint::FIXED: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::FIXED;
        break;
      }
      case urdf::Joint::CONTINUOUS:
      case urdf::Joint::REVOLUTE: {
        // TODO joint limit for revolute joint
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::REVOLUTE;    // since there's no joint limit, continuous joint = revolute joint
        break;
      }
      case urdf::Joint::PRISMATIC: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::PRISMATIC;
        break;
      }
      default:
       RSFATAL("currently only support revolute/prismatic/fixed joint");
    }

    double r, p, y;
    jnt->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
    benchmark::Vec<3> rpy;
    rpy.e() << r, p, y;
    double axisNorm = sqrt(jnt->axis.x * jnt->axis.x + jnt->axis.y * jnt->axis.y + jnt->axis.z * jnt->axis.z);
    childRef->parentJoint_.jointAxis({jnt->axis.x / axisNorm, jnt->axis.y / axisNorm, jnt->axis.z / axisNorm});
    childRef->parentJoint_.jointPosition({jnt->parent_to_joint_origin_transform.position.x,
                                          jnt->parent_to_joint_origin_transform.position.y,
                                          jnt->parent_to_joint_origin_transform.position.z});
    benchmark::rpyToRotMat_intrinsic(rpy, childRef->parentJoint_.rotmat_);

    processLinkFromUrdf(ch, *childRef, jointsOrder);
  }

   RSFATAL_IF(jointsNumber != raiLink.childrenLinks_.size(),
              "URDF reader error. Please report to eastsky.kang@gmail.com")
}

void OdeArticulatedSystem::init() {
  // init recursively
  benchmark::Mat<3,3> baseRotMat;
  baseRotMat.setIdentity();
  benchmark::Vec<3> baseOrigin;
  baseOrigin.setZero();

  // init index of each body
  initIdx(rootLink_);

  // init link
  initLink(rootLink_, baseRotMat, baseOrigin, visObj, visProps_, visColObj, visColProps_);

  // init ODE joints
  initJoints(rootLink_, baseRotMat, baseOrigin);

  // resize generalized states
  genForce_.resize(dof_);
  genForce_.setZero();
  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genCoordinate_.resize(stateDimension_);
  genCoordinate_.setZero();
}


void OdeArticulatedSystem::initIdx(Link &link) {
  link.bodyIdx_ = (int)links_.size();
  links_.push_back(&link);
  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    link.childrenLinks_[i].parentIdx_ = link.bodyIdx_;
    initIdx(link.childrenLinks_[i]);
  }
}

void OdeArticulatedSystem::initJoints(Link &link, benchmark::Mat<3, 3> &parentRot_w, benchmark::Vec<3> &parentPos_w) {

  if(link.bodyIdx_ == 0 && isFixed_) {
    // fixed base link
    rootJoint_.type = Joint::FIXED;
    rootJoint_.odeJoint_ = dJointCreateFixed(worldID_, 0);
    dJointAttach(rootJoint_.odeJoint_, link.odeBody_, 0);
    dJointSetFixed(rootJoint_.odeJoint_);
  }

  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    Link &childLink = link.childrenLinks_[i];

    // joint position, axis, and orientation
    benchmark::Vec<3> pos_w;
    benchmark::Vec<3> axis_w;
    benchmark::Vec<3> temp;
    benchmark::Mat<3,3> rot_w;

    // orientation
    benchmark::matmul(parentRot_w, childLink.parentJoint_.rotmat_, rot_w);

    // axis
    benchmark::matvecmul(childLink.parentJoint_.rotmat_, childLink.parentJoint_.axis_, temp);
    benchmark::matvecmul(parentRot_w, temp, axis_w);

    // position
    benchmark::matvecmul(parentRot_w, childLink.parentJoint_.pos_, pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    switch (childLink.parentJoint_.type) {
      case Joint::FIXED: {
        childLink.parentJoint_.odeJoint_ = dJointCreateFixed(worldID_, 0);
        dJointAttach(childLink.parentJoint_.odeJoint_, link.odeBody_, childLink.odeBody_);
        dJointSetFixed(childLink.parentJoint_.odeJoint_);
        break;
      }
      case Joint::REVOLUTE: {
        // joint
        childLink.parentJoint_.odeJoint_ = dJointCreateHinge(worldID_, 0);
        dJointAttach(childLink.parentJoint_.odeJoint_, link.odeBody_, childLink.odeBody_);
        dJointSetHingeAnchor(
            childLink.parentJoint_.odeJoint_,
            pos_w[0],
            pos_w[1],
            pos_w[2]
        );
        dJointSetHingeAxis(
            childLink.parentJoint_.odeJoint_,
            axis_w[0],
            axis_w[1],
            axis_w[2]
        );

        dof_++;
        stateDimension_++;
        break;
      }
      case Joint::PRISMATIC: {
        childLink.parentJoint_.odeJoint_ = dJointCreateSlider(worldID_, 0);
        dJointAttach(childLink.parentJoint_.odeJoint_, link.odeBody_, childLink.odeBody_);
        dJointSetSliderAxis(
            childLink.parentJoint_.odeJoint_,
            axis_w[0],
            axis_w[1],
            axis_w[2]
        );

        dof_++;
        stateDimension_++;
        break;
      }
      default:
       RSFATAL("currently only support revolute/prismatic/fixed joint");

    }

    // set data
    childLink.parentJoint_.childLink_ = &childLink;
    childLink.parentJoint_.gencoordIndex_ = stateDimension_ - 1;
    childLink.parentJoint_.genvelIndex_ = dof_ - 1;
    childLink.parentJoint_.jointId_ = (int)joints_.size();

    joints_.push_back(&childLink.parentJoint_);
    initJoints(childLink, rot_w, pos_w);
  }
}

void OdeArticulatedSystem::initLink(Link &link,
                                    benchmark::Mat<3, 3> &parentRot_w,
                                    benchmark::Vec<3> &parentPos_w,
                                    std::vector<VisualObjectData> &visualcollect,
                                    std::vector<VisualObjectProperty> &visualprops,
                                    std::vector<AlternativeVisualObjectData> &collisioncollect,
                                    std::vector<VisualObjectProperty> &collisionprops) {

  /*
   * NOTE
   * parentRot_w = joint orientation
   * parentPos_W = joint position
   */

  /// body
  {

    /*
     * body frame origin = COM (inertial frame)
     * body frame orientation = inertial frame orientation
     */

    // origin
    benchmark::Vec<3> bodyOrigin_w = parentPos_w;
    benchmark::matvecmulThenAdd(parentRot_w, link.inertial_.pos_, bodyOrigin_w);

    // orientation
    benchmark::Mat<3,3> bodyOrientation_w;
    benchmark::matmul(parentRot_w, link.inertial_.rotmat_, bodyOrientation_w);
    dMatrix3 drotation;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        drotation[4*i + j] = bodyOrientation_w[i+3*j];
      }
      drotation[4*i + 3] = 0;
    }
    dBodySetPosition(link.odeBody_, bodyOrigin_w[0], bodyOrigin_w[1], bodyOrigin_w[2]);
    dBodySetRotation(link.odeBody_, drotation);
    dBodySetGyroscopicMode(link.odeBody_, true);
  }

  /// inertial
  {
    /*
     * inertial frame origin = COM (inertial frame)
     * inertial frame orientation = inertial frame orientation
     */

    // origin and orientation
    benchmark::Vec<3> inertialOrigin_w;
    benchmark::Mat<3,3> inertialOrientation_w;

    if(link.inertial_.mass_ == 0) {
      // zero mass link
      if(link.bodyIdx_ == 0 && isFixed_) {
        // fixed base link (set default values)
        dMassSetParameters(
            &link.inertial_.odeMass_,
            1,
            0, 0, 0,
            1, 1, 1, 1, 1, 1
        );
        dBodySetMass(link.odeBody_, &link.inertial_.odeMass_);
      }
      else {
        // non-fixed-base link
         RSFATAL("zero inertial non-base link is not allowed in ODE")
      }
    } // end of zero mass link
    else {
      // non zero mass link
      benchmark::Mat<3,3> inertia = link.inertial_.inertia_;

      dMassSetParameters(
          &link.inertial_.odeMass_,
          link.inertial_.mass_,
          0, 0, 0,
          inertia[0], inertia[4], inertia[8], inertia[1], inertia[2], inertia[7]
      );

      // origin
      inertialOrigin_w = parentPos_w;
      benchmark::matvecmulThenAdd(parentRot_w, link.inertial_.pos_, inertialOrigin_w);

      // orientation
      benchmark::matmul(parentRot_w, link.inertial_.rotmat_, inertialOrientation_w);
      dMatrix3 drotation;
      for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
          drotation[4*i + j] = inertialOrientation_w[i+3*j];
        }
        drotation[4*i + 3] = 0;
      }
      dBodySetMass(link.odeBody_, &link.inertial_.odeMass_);
    }
  } // end of inertial

  /// collision
  {
    // collision objects
    for(int i = 0; i < link.collision_.colShape_.size(); i++) {

      /*
       * collision frame origin
       * collision frame orientation
       */

      benchmark::Vec<3> collisionOrigin_w;
      benchmark::Mat<3,3> collisionOrientation_w;

      benchmark::Vec<4> size = link.collision_.colShapeParam_[i];
      switch (link.collision_.colShape_[i]) {
        case bo::Shape::Cylinder: {
          link.collision_.odeGeometries_.push_back(dCreateCylinder(spaceID_, size[0], size[1]));
          dGeomSetCategoryBits(link.collision_.odeGeometries_.back(), collisionGroup_);
          dGeomSetCollideBits(link.collision_.odeGeometries_.back(), collisionMask_);
          break;
        }
        case bo::Shape::Sphere: {
          link.collision_.odeGeometries_.push_back(dCreateSphere(spaceID_, size[0]));
          dGeomSetCategoryBits(link.collision_.odeGeometries_.back(), collisionGroup_);
          dGeomSetCollideBits(link.collision_.odeGeometries_.back(), collisionMask_);
          break;
        }
        case bo::Shape::Box: {
          link.collision_.odeGeometries_.push_back(dCreateBox(spaceID_, size[0], size[1], size[2]));
          dGeomSetCategoryBits(link.collision_.odeGeometries_.back(), collisionGroup_);
          dGeomSetCollideBits(link.collision_.odeGeometries_.back(), collisionMask_);
          break;
        }
        default: {
           RSFATAL("this shape of collision is not supported yet")
          break;
        }
      }

      // set geometry properties
      link.collision_.matrialProps_.emplace_back();
      dGeomSetData(link.collision_.odeGeometries_.back(),
                   &link.collision_.matrialProps_.back());

      // origin
      collisionOrigin_w = parentPos_w;
      benchmark::matvecmulThenAdd(parentRot_w, link.collision_.colObjOrigin_[i], collisionOrigin_w);

      // orientation
      benchmark::matmul(parentRot_w, link.collision_.colObjRotMat_[i], collisionOrientation_w);

      // set each geometry position
      dMatrix3 geomR;
      for(int row = 0; row < 3; row++) {
        for(int col = 0; col < 3; col++) {
          geomR[4*row + col] = collisionOrientation_w[row+col*3];
        }
        geomR[4*row + 3] = 0;
      }

      // set geom orientation and position
      dGeomSetBody(link.collision_.odeGeometries_.back(), link.odeBody_);
      dGeomSetOffsetWorldPosition(link.collision_.odeGeometries_.back(),
                                  collisionOrigin_w[0],
                                  collisionOrigin_w[1],
                                  collisionOrigin_w[2]);
      dGeomSetOffsetWorldRotation(link.collision_.odeGeometries_.back(), geomR);

      // set alternative visualization object
      collisioncollect.emplace_back();
      collisioncollect.back() = std::make_tuple(link.collision_.colObjRotMat_[i],
                                                link.collision_.colObjOrigin_[i],
                                                link.bodyIdx_,
                                                link.collision_.colShape_[i]);
      collisionprops.emplace_back("",
                                  link.collision_.colShapeParam_[i]);
    } // end of geometry
  } // end of collision

  /// visual
  {
    // visual objects
    for(int i = 0; i < link.visual_.visshape_.size(); i++) {
      visualcollect.emplace_back();
      visualcollect.back() = std::make_tuple(link.visual_.visObjRotMat_[i],
                                             link.visual_.visObjOrigin_[i],
                                             link.bodyIdx_,
                                             link.visual_.visshape_[i],
                                             link.visual_.visColor_[i]);
      visualprops.emplace_back(link.visual_.meshFileNames_[i],
                               link.visual_.visShapeParam_[i]);
    }
  }

  /// children
  for (auto &ch: link.childrenLinks_) {
    benchmark::Mat<3,3> rot_w;
    benchmark::Vec<3> pos_w = parentPos_w;

    benchmark::matmul(parentRot_w, ch.parentJoint_.rotmat_, rot_w);
    benchmark::matvecmulThenAdd(parentRot_w, ch.parentJoint_.pos_, pos_w);

    initLink(ch, rot_w, pos_w, visualcollect, visualprops, collisioncollect, collisionprops);
  }
}

OdeArticulatedSystem::~OdeArticulatedSystem() {
  for(int i = 0; i < links_.size(); i++) {

    // delete ode joints
    if(links_[i]->parentJoint_.odeJoint_)
      dJointDestroy(links_[i]->parentJoint_.odeJoint_);

    // delete ode body
    if(links_[i]->odeBody_)
      dBodyDestroy(links_[i]->odeBody_);

    // delete ode geom
    if(links_[i]->collision_.odeGeometries_.size() > 0) {
      for(int j = 0; j < links_[i]->collision_.odeGeometries_.size(); j++)
        dGeomDestroy(links_[i]->collision_.odeGeometries_[j]);
    }
  }
}

int OdeArticulatedSystem::getDOF() {
  return dof_;
}

int OdeArticulatedSystem::getStateDimension() {
  return stateDimension_;
}

void OdeArticulatedSystem::updateBodyPos(Link &link,
                                         benchmark::Mat<3, 3> &parentRot_w,
                                         benchmark::Vec<3> &parentPos_w) {

  {
    /*
     * body frame origin = COM (inertial frame)
     * body frame orientation = inertial frame orientation
     */

    // origin
    benchmark::Vec<3> bodyOrigin_w = parentPos_w;
    benchmark::matvecmulThenAdd(parentRot_w, link.inertial_.pos_, bodyOrigin_w);

    // orientation
    benchmark::Mat<3, 3> bodyOrientation_w;
    benchmark::matmul(parentRot_w, link.inertial_.rotmat_, bodyOrientation_w);
    dMatrix3 drotation;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        drotation[4 * i + j] = bodyOrientation_w[i + 3 * j];
      }
      drotation[4 * i + 3] = 0;
    }
    dBodySetPosition(link.odeBody_, bodyOrigin_w[0], bodyOrigin_w[1], bodyOrigin_w[2]);
    dBodySetRotation(link.odeBody_, drotation);
  }

  // children
  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    Link &childLink = link.childrenLinks_[i];

    // joint position, axis, and orientation
    benchmark::Vec<3> pos_w;
    benchmark::Vec<3> axis_w;
    benchmark::Vec<3> tempV;
    benchmark::Mat<3,3> rot_w;
    benchmark::Mat<3,3> jointR;
    benchmark::Mat<3,3> tempR;

    switch (childLink.parentJoint_.type) {
      case Joint::FIXED: {
        benchmark::matmul(parentRot_w, childLink.parentJoint_.rotmat_, rot_w);
        benchmark::matvecmul(parentRot_w, childLink.parentJoint_.pos_, pos_w);
        benchmark::vecadd(parentPos_w, pos_w);
        break;
      }
      case Joint::REVOLUTE: {
        // orientation
        benchmark::angleAxisToRotMat(
            childLink.parentJoint_.axis_,
            genCoordinate_[childLink.parentJoint_.gencoordIndex_],
            jointR
        );
        benchmark::matmul(childLink.parentJoint_.rotmat_, jointR, tempR);
        benchmark::matmul(parentRot_w, tempR, rot_w);

        // axis
        benchmark::matvecmul(childLink.parentJoint_.rotmat_, childLink.parentJoint_.axis_, tempV);
        benchmark::matvecmul(parentRot_w, tempV, axis_w);

        // position
        benchmark::matvecmul(parentRot_w, childLink.parentJoint_.pos_, pos_w);
        benchmark::vecadd(parentPos_w, pos_w);
        break;
      }
      case Joint::PRISMATIC: {
        // orientation
        benchmark::matmul(parentRot_w, childLink.parentJoint_.rotmat_, rot_w);

        // axis
        benchmark::Vec<3> tempV2;
        benchmark::matvecmul(childLink.parentJoint_.rotmat_, childLink.parentJoint_.axis_, tempV2);

        // position
        benchmark::vecScalarMul(
            genCoordinate_[childLink.parentJoint_.gencoordIndex_],
            tempV2,
            tempV
        );

        benchmark::vecadd(childLink.parentJoint_.pos_, tempV);

        benchmark::matvecmul(parentRot_w, tempV, pos_w);
        benchmark::vecadd(parentPos_w, pos_w);
        break;
      }
      default:
       RSFATAL("currently only support revolute/prismatic/fixed joint");
    }

    updateBodyPos(childLink, rot_w, pos_w);
  }
}

const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedCoordinate() {
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genCoordinate_[i++] = -dJointGetHingeAngle(joint->odeJoint_);;
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          genCoordinate_[i++] = dJointGetSliderPosition(joint->odeJoint_);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    const dReal *position = dBodyGetPosition(rootLink_.odeBody_);
    const dReal *quaternion = dBodyGetQuaternion(rootLink_.odeBody_);

    genCoordinate_[0] = position[0];
    genCoordinate_[1] = position[1];
    genCoordinate_[2] = position[2];
    genCoordinate_[3] = quaternion[0];
    genCoordinate_[4] = quaternion[1];
    genCoordinate_[5] = quaternion[2];
    genCoordinate_[6] = quaternion[3];

    int i = 7;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genCoordinate_[i++] = -dJointGetHingeAngle(joint->odeJoint_);
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          genCoordinate_[i++] = dJointGetSliderPosition(joint->odeJoint_);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
  return genCoordinate_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedVelocity() {
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genVelocity_[i++] = -dJointGetHingeAngleRate(joint->odeJoint_);;
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          genVelocity_[i++] = dJointGetSliderPositionRate(joint->odeJoint_);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    const dReal *linvel = dBodyGetLinearVel(rootLink_.odeBody_);
    const dReal *angvel = dBodyGetAngularVel(rootLink_.odeBody_);

    genVelocity_[0] = linvel[0];
    genVelocity_[1] = linvel[1];
    genVelocity_[2] = linvel[2];
    genVelocity_[3] = angvel[0];
    genVelocity_[4] = angvel[1];
    genVelocity_[5] = angvel[2];

    int i = 6;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genVelocity_[i++] = -dJointGetHingeAngleRate(joint->odeJoint_);;
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          genVelocity_[i++] = dJointGetSliderPositionRate(joint->odeJoint_);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
  return genVelocity_.e();
}

void OdeArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
   RSFATAL("not implemented yet")
}

void OdeArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
   RSFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")

  for(int i = 0; i < stateDimension_; i++) {
    genCoordinate_[i] = jointState[i];
  }

  if(isFixed_) {
    // fixed body
    benchmark::Mat<3,3> baseRotMat;
    baseRotMat.setIdentity();
    benchmark::Vec<3> baseOrigin;
    baseOrigin.setZero();

    updateBodyPos(rootLink_, baseRotMat, baseOrigin);
  }
  else {
    // floating body
    benchmark::Mat<3,3> baseRotMat;
    benchmark::Vec<4> baseQuat;
    baseQuat = {
        jointState[3],
        jointState[4],
        jointState[5],
        jointState[6],
    };
    benchmark::quatToRotMat(baseQuat, baseRotMat);

    benchmark::Vec<3> baseOrigin;
    baseOrigin = {
        jointState[0],
        jointState[1],
        jointState[2]
    };

    updateBodyPos(rootLink_, baseRotMat, baseOrigin);
  }
}

void OdeArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
   RSFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")

  for(int i = 0; i < stateDimension_; i++) {
    genCoordinate_[i] = jointState.begin()[i];
  }

  if(isFixed_) {
    // fixed body
    benchmark::Mat<3,3> baseRotMat;
    baseRotMat.setIdentity();
    benchmark::Vec<3> baseOrigin;
    baseOrigin.setZero();

    updateBodyPos(rootLink_, baseRotMat, baseOrigin);
  }
  else {
    // floating body
    benchmark::Mat<3,3> baseRotMat;
    benchmark::Vec<4> baseQuat;
    baseQuat = {
        jointState.begin()[3],
        jointState.begin()[4],
        jointState.begin()[5],
        jointState.begin()[6],
    };
    benchmark::quatToRotMat(baseQuat, baseRotMat);

    benchmark::Vec<3> baseOrigin;
    baseOrigin = {
        jointState.begin()[0],
        jointState.begin()[1],
        jointState.begin()[2]
    };

    updateBodyPos(rootLink_, baseRotMat, baseOrigin);
  }
}

void OdeArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
   RSFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau.begin()[i++]);
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    dBodyAddForce(rootLink_.odeBody_, tau.begin()[0], tau.begin()[1], tau.begin()[2]);
    dBodyAddTorque(rootLink_.odeBody_, tau.begin()[3], tau.begin()[4], tau.begin()[4]);

    int i = 6;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau.begin()[i++]);
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
}

void OdeArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
   RSFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau[i++]);
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau[i++]);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    dBodyAddForce(rootLink_.odeBody_, tau[0], tau[1], tau[2]);
    dBodyAddTorque(rootLink_.odeBody_, tau[3], tau[4], tau[4]);

    int i = 6;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau[i++]);
          break;
        }
        case Joint::PRISMATIC: {
           RSFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau[i++]);
          break;
        }
        default:
         RSINFO("not supported joint type")
      }
    }
  }
}

void OdeArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  setGeneralizedCoordinate(genco);
  setGeneralizedVelocity(genvel);
}

const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedForce() {
   RSFATAL("not implemented yet")
  return genForce_.e();
}

const std::vector<Joint *> &OdeArticulatedSystem::getJoints() const {
  return joints_;
}

const std::vector<Link *> &OdeArticulatedSystem::getLinks() const {
  return links_;
}

void OdeArticulatedSystem::getBodyPose(int bodyId, benchmark::Mat<3, 3> &orientation, benchmark::Vec<3> &position) {
  Link *link = links_[bodyId];

  // com position w.r.t. world
  const dReal *pos = dBodyGetPosition(link->odeBody_);
  position = {pos[0], pos[1], pos[2]};

  // body rotation w.r.t. world
  const dReal* rot = dBodyGetRotation(link->odeBody_);
  benchmark::Mat<3,3> tempMat;
  tempMat.e() << rot[0], rot[1], rot[2],
      rot[4], rot[5], rot[6],
      rot[8], rot[9], rot[10];

  // body position (joint position)
  benchmark::Vec<3> tempVec;
  benchmark::matvecmul(tempMat, link->inertial_.pos_, tempVec);
  benchmark::vecsub(tempVec, position);

  // body orientation (joint orientation)
  benchmark::transposed2MatMul(tempMat, link->inertial_.rotmat_, orientation);
}

void OdeArticulatedSystem::getComVelocity_W(int bodyId, benchmark::Vec<3> &velocity) {
  const dReal *comvel = dBodyGetLinearVel(links_[bodyId]->odeBody_);
  velocity = {comvel[0], comvel[1], comvel[2]};
}

void OdeArticulatedSystem::getBodyOmega_W(int bodyId, benchmark::Vec<3> &omega) {
  const dReal *angvel = dBodyGetAngularVel(links_[bodyId]->odeBody_);
  omega = {angvel[0], angvel[1], angvel[2]};
}

void OdeArticulatedSystem::getComPos_W(int bodyId, benchmark::Vec<3> &comPos) {
  const dReal *compos = dBodyGetPosition(links_[bodyId]->odeBody_);
  comPos = {compos[0], compos[1], compos[2]};
}

void OdeArticulatedSystem::getComRot_W(int bodyId, benchmark::Mat<3, 3> &comOrientation) {
  const dReal *comR = dBodyGetRotation(links_[bodyId]->odeBody_);
  comOrientation.e() << comR[0], comR[1], comR[2],
      comR[4], comR[5], comR[6],
      comR[8], comR[9], comR[10];
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> OdeArticulatedSystem::getLinearMomentum() {
  linearMomentum_.setZero();
  for(int i = 0; i < links_.size(); i++) {
    benchmark::Vec<3> comvel;
    getComVelocity_W(i, comvel);
    benchmark::vecScalarMul(links_[i]->inertial_.odeMass_.mass, comvel);
    benchmark::vecadd(comvel, linearMomentum_);
  }
  return linearMomentum_.e();
}

double OdeArticulatedSystem::getTotalMass() {
  double totalMass = 0;
  for(int i = 0; i < links_.size(); i++) {
    totalMass += links_[i]->inertial_.odeMass_.mass;
  }
  return totalMass;
}

double OdeArticulatedSystem::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
}

double OdeArticulatedSystem::getKineticEnergy() {
  double kinetic = 0;
  for(int i = 0; i < links_.size(); i++) {
    benchmark::Vec<3> comvel;
    getComVelocity_W(i, comvel);
    kinetic += pow(comvel.norm(), 2) * links_[i]->inertial_.odeMass_.mass;

    double angular;
    benchmark::Mat<3,3> orientation_w;
    getComRot_W(i, orientation_w);

    benchmark::Mat<3,3> I_w;
    benchmark::similarityTransform(orientation_w, links_[i]->inertial_.inertia_, I_w);

    benchmark::Vec<3> omega;  // angvel of inertial frame
    getBodyOmega_W(i, omega);
    benchmark::vecTransposeMatVecMul(omega, I_w, angular);
    kinetic += angular;
  }

  return 0.5 * kinetic;
}

double OdeArticulatedSystem::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  double potential = 0;
  for(int i = 0; i < links_.size(); i++) {
    double linkPotential = 0;
    benchmark::Vec<3> temp;
    getComPos_W(i, temp);
    benchmark::vecDot(gravity, temp, linkPotential);
    potential -= linkPotential * links_[i]->inertial_.mass_;
  }

  return potential;
}


void OdeArticulatedSystem::updateBodyVelocity(Link &link,
                                              benchmark::Vec<3> &parentAngVel_w,
                                              benchmark::Vec<3> &parentLinVel_w) {
  /*
  * body frame origin = COM (inertial frame)
  * body frame orientation = inertial frame orientation
  */

  benchmark::Mat<3,3> parentOrientation_w;
  benchmark::Vec<3> parentPosition_w;
  getBodyPose(link.bodyIdx_, parentOrientation_w, parentPosition_w);

  benchmark::Vec<3> parent2com_w;
  getComPos_W(link.bodyIdx_, parent2com_w);
  benchmark::vecsub(parentPosition_w, parent2com_w);

  // {w}_bodylinvel = {w}_parentlinvel + {w}_w x {w}_r
  benchmark::Vec<3> bodyLinVel_w = parentLinVel_w;
  benchmark::crossThenAdd(parentAngVel_w, parent2com_w, bodyLinVel_w);

  // {w}_bodyangvel = R_{com}_{w} * {w}_w
  benchmark::Mat<3, 3> comOrientation_w;
  getComRot_W(link.bodyIdx_, comOrientation_w);

  benchmark::Vec<3> bodyAngVel_w;
  benchmark::matTransposevecmul(comOrientation_w, parentAngVel_w, bodyAngVel_w);

  dBodySetLinearVel(link.odeBody_, bodyLinVel_w[0], bodyLinVel_w[1], bodyLinVel_w[2]);
  dBodySetAngularVel(link.odeBody_, bodyAngVel_w[0], bodyAngVel_w[1], bodyAngVel_w[2]);

  // children
  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    Link &childLink = link.childrenLinks_[i];

    // joint position, axis, and orientation
    benchmark::Vec<3> jointLinVel_w;
    benchmark::Vec<3> jointAngVel_w;

    switch (childLink.parentJoint_.type) {
      case Joint::FIXED: {
        jointAngVel_w = parentAngVel_w;

        // {w}_bodylinvel = {w}_parentlinvel + {w}_w x {w}_r
        benchmark::Mat<3,3> dummy;
        benchmark::Vec<3> parent2joint_w;
        getBodyPose(childLink.bodyIdx_, dummy, parent2joint_w);
        benchmark::vecsub(parentPosition_w, parent2joint_w);

        jointLinVel_w = parentLinVel_w;
        benchmark::crossThenAdd(parentAngVel_w, parent2joint_w, jointLinVel_w);
        break;
      }
      case Joint::REVOLUTE: {
        double rate = genVelocity_[childLink.parentJoint_.genvelIndex_];
        dReal axis[3];
        dJointGetHingeAxis(childLink.parentJoint_.odeJoint_, axis);

        // ang vel
        jointAngVel_w[0] = parentAngVel_w[0] + axis[0] * rate;
        jointAngVel_w[1] = parentAngVel_w[1] + axis[1] * rate;
        jointAngVel_w[2] = parentAngVel_w[2] + axis[2] * rate;

        // {w}_jointlinvel = {w}_parentlinvel + {w}_w x {w}_r
        benchmark::Mat<3,3> dummy;
        benchmark::Vec<3> parent2joint_w;
        getBodyPose(childLink.bodyIdx_, dummy, parent2joint_w);
        benchmark::vecsub(parentPosition_w, parent2joint_w);

        jointLinVel_w = parentLinVel_w;
        benchmark::crossThenAdd(parentAngVel_w, parent2joint_w, jointLinVel_w);
        break;
      }
      case Joint::PRISMATIC: {
        double rate = genVelocity_[childLink.parentJoint_.genvelIndex_];
         RSFATAL("not implemented yet");
        break;
      }
      default:
       RSFATAL("currently only support revolute/prismatic/fixed joint");
    }

    updateBodyVelocity(childLink, jointAngVel_w, jointLinVel_w);
  }


}

void OdeArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
   RSFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  RSWARN("direct assigning joint velocity is not tested!")

  for(int i = 0; i < dof_; i++)
    genVelocity_[i]= jointVel[i];

  if(isFixed_) {
    // fixed body
    benchmark::Vec<3> baseLinVel;
    benchmark::Vec<3> baseAngVel;
    baseLinVel = {0, 0, 0};
    baseAngVel = {0, 0, 0};
    updateBodyVelocity(rootLink_, baseAngVel, baseLinVel);
  }
  else {
    // floating body
    benchmark::Vec<3> baseLinVel;
    benchmark::Vec<3> baseAngVel;
    baseLinVel = {
        jointVel[0],
        jointVel[1],
        jointVel[2]
    };
    baseAngVel = {
        jointVel[3],
        jointVel[4],
        jointVel[5]
    };
    updateBodyVelocity(rootLink_, baseAngVel, baseLinVel);
  }
}
void OdeArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
   RSFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  RSWARN("direct assigning joint velocity is not tested!")

  for(int i = 0; i < dof_; i++)
    genVelocity_[i]= jointVel.begin()[i];

  if(isFixed_) {
    // fixed body
    benchmark::Vec<3> baseLinVel;
    benchmark::Vec<3> baseAngVel;
    baseLinVel = {0, 0, 0};
    baseAngVel = {0, 0, 0};
    updateBodyVelocity(rootLink_, baseAngVel, baseLinVel);
  }
  else {
    // floating body
    benchmark::Vec<3> baseLinVel;
    benchmark::Vec<3> baseAngVel;
    baseLinVel = {
        jointVel.begin()[0],
        jointVel.begin()[1],
        jointVel.begin()[2]
    };
    baseAngVel = {
        jointVel.begin()[3],
        jointVel.begin()[4],
        jointVel.begin()[5]
    };
    updateBodyVelocity(rootLink_, baseAngVel, baseLinVel);
  }
}

} // object
} // ode_sim