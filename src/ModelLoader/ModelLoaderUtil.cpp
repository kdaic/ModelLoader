/*
 * Copyright (c) 2019, kdaic
 * Original source:
 *   https://github.com/fkanehiro/openhrp3/blob/3.1.9/hrplib/hrpModel/ModelLoaderUtil.cpp
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/**
   @author Shin'ichiro Nakaoka
   @author Ergovision
   @author kdaic
*/

#include <stack>

#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/ColdetModel.h>

#include "Link.h"
#include "Sensor.h"
#include "Light.h"

#include "ModelLoader.hpp"
#include "ModelLoaderUtil.hpp"

using namespace std;
using namespace hrp;

namespace {

const bool debugMode = false;

///////////////////////////////////////////////////////////////////////////////////////////

namespace
{

static Link *createNewLink() { return new Link(); }

class ModelLoaderHelper
{
public:
  ModelLoaderHelper() {
    collisionDetectionModelLoading_ = false;
    createLinkFunc_ = createNewLink;
  }

  void enableCollisionDetectionModelLoading(bool isEnabled) {
    collisionDetectionModelLoading_ = isEnabled;
  };
  void setLinkFactory(Link *(*f)()) { createLinkFunc_ = f; }

  bool createBody(BodyPtr& body,  BodyInfo_ptr bodyInfo);

private:
  BodyPtr body_;
  std::vector<LinkInfo> linkInfoSeq_;
  std::vector<ShapeInfo> shapeInfoSeq_;
  std::vector<ExtraJointInfo> extraJointInfoSeq_;
  bool collisionDetectionModelLoading_;
  Link *(*createLinkFunc_)();

  Link* createLink(int& index, const Matrix33& parentRs);
  void createSensors(Link* link, const SensorInfoSequence& sensorInfoSeq, const Matrix33& Rs);
  void createLights(Link* link, const LightInfoSequence& lightInfoSeq, const Matrix33& Rs);
  void createColdetModel(Link* link, const LinkInfo& linkInfo);
  void addLinkPrimitiveInfo(ColdetModelPtr& coldetModel,
                            const double *R, const double *p,
                            const ShapeInfo& shapeInfo);
  void addLinkVerticesAndTriangles(ColdetModelPtr& coldetModel, const LinkInfo& linkInfo);
  void addLinkVerticesAndTriangles(ColdetModelPtr& coldetModel, const TransformedShapeIndex& tsi, const Matrix44& Tparent, ShapeInfoSequence& shapes, int& vertexIndex, int& triangleIndex);
  void setExtraJoints();
};

};

///////////////////////////////////////////////////////////////////////////////////////////


inline double g_getLimitValue(std::vector<double> limitseq, double defaultValue)
{
  return (limitseq.size() == 0) ? defaultValue : limitseq[0];
}


};

///////////////////////////////////////////////////////////////////////////////////////////


bool ModelLoaderHelper::createBody(BodyPtr& body, BodyInfo_ptr bodyInfo)
{
  this->body_ = body;

  std::string name = bodyInfo->name();
  body_->setModelName(name);
  body_->setName(name);

  int n = bodyInfo->links().size();
  linkInfoSeq_ = bodyInfo->links();
  shapeInfoSeq_ = bodyInfo->shapes();
  extraJointInfoSeq_ = bodyInfo->extraJoints();

  int rootIndex = -1;

  for(int i=0; i < n; ++i){
    if(linkInfoSeq_[i].parentIndex < 0){
      if(rootIndex < 0){
        rootIndex = i;
      } else {
        // more than one root !
        rootIndex = -1;
        break;
      }
    }
  }

  if(rootIndex >= 0){ // root exists

    Matrix33 Rs(Matrix33::Identity());
    Link* rootLink = createLink(rootIndex, Rs);
    body_->setRootLink(rootLink);
    body_->setDefaultRootPosition(rootLink->b, rootLink->Rs);

    body_->installCustomizer();
    body_->initializeConfiguration();

    setExtraJoints();

    return true;
  }

  return false;
}


Link* ModelLoaderHelper::createLink(int& index, const Matrix33& parentRs)
{
  // std::cout << "linkInfo index : " << index << std::endl;
  LinkInfo& linkInfo = linkInfoSeq_[index];

  int jointId = linkInfo.jointId;

  Link* link = (*createLinkFunc_)();

  std::string name0 = linkInfo.name;
  link->name = string( name0 );
  link->jointId = jointId;

  Vector3 relPos(linkInfo.translation[0], linkInfo.translation[1], linkInfo.translation[2]);
  link->b = parentRs * relPos;

  Vector3 rotAxis(linkInfo.rotation[0], linkInfo.rotation[1], linkInfo.rotation[2]);
  Matrix33 R = rodrigues(rotAxis, linkInfo.rotation[3]);
  link->Rs = (parentRs * R);
  const Matrix33& Rs = link->Rs;

  std::string jointType = linkInfo.jointType;
  const std::string jt( jointType );

  if(jt == "fixed" ){
    link->jointType = Link::FIXED_JOINT;
  } else if(jt == "free" ){
    link->jointType = Link::FREE_JOINT;
  } else if(jt == "rotate" ){
    link->jointType = Link::ROTATIONAL_JOINT;
  } else if(jt == "slide" ){
    link->jointType = Link::SLIDE_JOINT;
  } else if(jt == "crawler"){
    link->jointType = Link::FIXED_JOINT;
    link->isCrawler = true;
  } else {
    link->jointType = Link::FREE_JOINT;
  }

  if(jointId < 0){
    if(link->jointType == Link::ROTATIONAL_JOINT || link->jointType == Link::SLIDE_JOINT){
      std::cerr << "Warning:  Joint ID is not given to joint " << link->name
                << " of model " << body_->modelName() << "." << std::endl;
    }
  }

  link->a.setZero();
  link->d.setZero();

  Vector3 axis( Rs * Vector3(linkInfo.jointAxis[0], linkInfo.jointAxis[1], linkInfo.jointAxis[2]));

  if(link->jointType == Link::ROTATIONAL_JOINT || jt == "crawler"){
    link->a = axis;
  } else if(link->jointType == Link::SLIDE_JOINT){
    link->d = axis;
  }

  link->m  = linkInfo.mass;
  link->Ir = linkInfo.rotorInertia;

  link->gearRatio     = linkInfo.gearRatio;
  link->rotorResistor = linkInfo.rotorResistor;
  link->torqueConst   = linkInfo.torqueConst;
  link->encoderPulse  = linkInfo.encoderPulse;

  link->Jm2 = link->Ir * link->gearRatio * link->gearRatio;

  std::vector<double> ulimit  = linkInfo.ulimit;
  std::vector<double> llimit  = linkInfo.llimit;
  std::vector<double> uvlimit = linkInfo.uvlimit;
  std::vector<double> lvlimit = linkInfo.lvlimit;
  std::vector<double> climit  = linkInfo.climit;

  double maxlimit = (numeric_limits<double>::max)();

  link->ulimit  = g_getLimitValue(ulimit,  +maxlimit);
  link->llimit  = g_getLimitValue(llimit,  -maxlimit);
  link->uvlimit = g_getLimitValue(uvlimit, +maxlimit);
  link->lvlimit = g_getLimitValue(lvlimit, -maxlimit);
  link->climit  = g_getLimitValue(climit,  +maxlimit);

  link->c = Rs * Vector3(linkInfo.centerOfMass[0], linkInfo.centerOfMass[1], linkInfo.centerOfMass[2]);

  Matrix33 Io;
  getMatrix33FromRowMajorArray(Io, linkInfo.inertia);
  link->I = Rs * Io * Rs.transpose();

  // a stack is used for keeping the same order of children
  std::stack<Link*> children;

  //##### [Changed] Link Structure (convert NaryTree to BinaryTree).
  int childNum = linkInfo.childIndices.size();
  for(int i=0;  i < childNum; i++ ) {
    int childIndex = linkInfo.childIndices[i];
    Link* childLink = createLink(childIndex, Rs);
    if(childLink) {
      children.push(childLink);
    }
  }

  while(!children.empty()){
    link->addChild(children.top());
    children.pop();
  }

  createSensors(link, linkInfo.sensors, Rs);
  createLights(link, linkInfo.lights, Rs);

  if(collisionDetectionModelLoading_){
    createColdetModel(link, linkInfo);
  }

  return link;
}


void ModelLoaderHelper::createLights(Link* link, const LightInfoSequence& lightInfoSeq, const Matrix33& Rs)
{
  int numLights = lightInfoSeq.size();

  for(int i=0 ; i < numLights ; ++i ) {
    const LightInfo& lightInfo = lightInfoSeq[i];
    std::string name(lightInfo.name);
    Light *light = body_->createLight(link, lightInfo.type, name);
    const boost::array<double,12>& T = lightInfo.transformMatrix;
    light->localPos << T[3], T[7], T[11];
    light->localR << T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10];
    switch (lightInfo.type){
    case Light::POINT:
      light->ambientIntensity = lightInfo.ambientIntensity;
      getVector3(light->attenuation, lightInfo.attenuation);
      getVector3(light->color, lightInfo.color);
      light->intensity = lightInfo.intensity;
      getVector3(light->location, lightInfo.location);
      light->on = lightInfo.on;
      light->radius = lightInfo.radius;
      break;
    case Light::DIRECTIONAL:
      light->ambientIntensity = lightInfo.ambientIntensity;
      getVector3(light->color, lightInfo.color);
      light->intensity = lightInfo.intensity;
      light->on = lightInfo.on;
      getVector3(light->direction, lightInfo.color);
      break;
    case Light::SPOT:
      light->ambientIntensity = lightInfo.ambientIntensity;
      getVector3(light->attenuation, lightInfo.attenuation);
      getVector3(light->color, lightInfo.color);
      light->intensity = lightInfo.intensity;
      getVector3(light->location, lightInfo.location);
      light->on = lightInfo.on;
      light->radius = lightInfo.radius;
      getVector3(light->direction, lightInfo.direction);
      light->beamWidth = lightInfo.beamWidth;
      light->cutOffAngle = lightInfo.cutOffAngle;
      break;
    default:
      std::cerr << "unknown light type" << std::endl;
    }
  }
}

void ModelLoaderHelper::createSensors(Link* link, const SensorInfoSequence& sensorInfoSeq, const Matrix33& Rs)
{
  int numSensors = sensorInfoSeq.size();

  for(int i=0 ; i < numSensors ; ++i ) {
    const SensorInfo& sensorInfo = sensorInfoSeq[i];

    int id = sensorInfo.id;
    if(id < 0) {
      std::cerr << "Warning:  sensor ID is not given to sensor " << sensorInfo.name
                << "of model " << body_->modelName() << "." << std::endl;
    } else {
      int sensorType = Sensor::COMMON;

      std::string type0 = sensorInfo.type;
      std::string type(type0);

      if(type == "Force")             { sensorType = Sensor::FORCE; }
      else if(type == "RateGyro")     { sensorType = Sensor::RATE_GYRO; }
      else if(type == "Acceleration")	{ sensorType = Sensor::ACCELERATION; }
      else if(type == "Vision")       { sensorType = Sensor::VISION; }
      else if(type == "Range")        { sensorType = Sensor::RANGE; }

      std::string name0 = sensorInfo.name;
      std::string name(name0);

      Sensor* sensor = body_->createSensor(link, sensorType, id, name);

      if(sensor) {
        const boost::array<double,3> p = sensorInfo.translation;
        sensor->localPos = Rs * Vector3(p[0], p[1], p[2]);

        const Vector3 axis(sensorInfo.rotation[0], sensorInfo.rotation[1], sensorInfo.rotation[2]);
        const Matrix33 R(rodrigues(axis, sensorInfo.rotation[3]));
        sensor->localR = Rs * R;
      }

      if ( sensorType == Sensor::RANGE ) {
        RangeSensor *range = dynamic_cast<RangeSensor *>(sensor);
        range->scanAngle = sensorInfo.specValues[0];
        range->scanStep = sensorInfo.specValues[1];
        range->scanRate = sensorInfo.specValues[2];
        range->maxDistance = sensorInfo.specValues[3];
      }else if (sensorType == Sensor::VISION) {
        VisionSensor *vision = dynamic_cast<VisionSensor *>(sensor);
        vision->near   = sensorInfo.specValues[0];
        vision->far    = sensorInfo.specValues[1];
        vision->fovy   = sensorInfo.specValues[2];
        vision->width  = sensorInfo.specValues[4];
        vision->height = sensorInfo.specValues[5];
        int npixel = vision->width*vision->height;
        switch((int)sensorInfo.specValues[3]){
        case Camera::NONE:
          vision->imageType = VisionSensor::NONE;
          break;
        case Camera::COLOR:
          vision->imageType = VisionSensor::COLOR;
          vision->image.resize(npixel*3);
          break;
        case Camera::MONO:
          vision->imageType = VisionSensor::MONO;
          vision->image.resize(npixel);
          break;
        case Camera::DEPTH:
          vision->imageType = VisionSensor::DEPTH;
          break;
        case Camera::COLOR_DEPTH:
          vision->imageType = VisionSensor::COLOR_DEPTH;
          vision->image.resize(npixel*3);
          break;
        case Camera::MONO_DEPTH:
          vision->imageType = VisionSensor::MONO_DEPTH;
          vision->image.resize(npixel);
          break;
        }
        vision->frameRate = sensorInfo.specValues[6];
      }
    }
  }
}


void ModelLoaderHelper::createColdetModel(Link* link, const LinkInfo& linkInfo)
{
  int totalNumVertices = 0;
  int totalNumTriangles = 0;
  const TransformedShapeIndexSequence& shapeIndices = linkInfo.shapeIndices;
  unsigned int nshape = shapeIndices.size();
  short shapeIndex;
  double R[9], p[3];
  for(unsigned int i=0; i < shapeIndices.size(); i++){
    shapeIndex = shapeIndices[i].shapeIndex;
    const boost::array<double,12>& tform = shapeIndices[i].transformMatrix;
    R[0] = tform[0]; R[1] = tform[1]; R[2] = tform[2]; p[0] = tform[3];
    R[3] = tform[4]; R[4] = tform[5]; R[5] = tform[6]; p[1] = tform[7];
    R[6] = tform[8]; R[7] = tform[9]; R[8] = tform[10]; p[2] = tform[11];
    const ShapeInfo& shapeInfo = shapeInfoSeq_[shapeIndex];
    totalNumVertices += shapeInfo.vertices.size() / 3;
    totalNumTriangles += shapeInfo.triangles.size() / 3;
  }

  const SensorInfoSequence& sensors = linkInfo.sensors;
  for (unsigned int i=0; i<sensors.size(); i++){
    const SensorInfo &sinfo = sensors[i];
    const TransformedShapeIndexSequence tsis = sinfo.shapeIndices;
    nshape += tsis.size();
    for (unsigned int j=0; j<tsis.size(); j++){
      short shapeIndex = tsis[j].shapeIndex;
      const ShapeInfo& shapeInfo = shapeInfoSeq_[shapeIndex];
      totalNumTriangles += shapeInfo.triangles.size() / 3;
      totalNumVertices += shapeInfo.vertices.size() / 3 ;
    }
  }

  ColdetModelPtr coldetModel(new ColdetModel());
  coldetModel->setName(std::string(linkInfo.name));
  if(totalNumTriangles > 0){
    coldetModel->setNumVertices(totalNumVertices);
    coldetModel->setNumTriangles(totalNumTriangles);
    if (nshape == 1){
      addLinkPrimitiveInfo(coldetModel, R, p, shapeInfoSeq_[shapeIndex]);
    }
    addLinkVerticesAndTriangles(coldetModel, linkInfo);
    coldetModel->build();
  }
  link->coldetModel = coldetModel;
}

void ModelLoaderHelper::addLinkVerticesAndTriangles
(ColdetModelPtr& coldetModel, const TransformedShapeIndex& tsi, const Matrix44& Tparent, std::vector<ShapeInfo>& shapes, int& vertexIndex, int& triangleIndex)
{
  short shapeIndex = tsi.shapeIndex;
  const boost::array<double,12> M = tsi.transformMatrix;
  Matrix44 T, Tlocal;
  Tlocal << M[0], M[1], M[2],  M[3],
    M[4], M[5], M[6],  M[7],
    M[8], M[9], M[10], M[11],
    0.0,  0.0,  0.0,   1.0;
  T = Tparent * Tlocal;

  const ShapeInfo& shapeInfo = shapes[shapeIndex];
  int vertexIndexBase = vertexIndex;
  const std::vector<double>& vertices = shapeInfo.vertices;
  const int numVertices = vertices.size() / 3;
  for(int j=0; j < numVertices; ++j){
    Vector4 v(T * Vector4(vertices[j*3], vertices[j*3+1], vertices[j*3+2], 1.0));
    coldetModel->setVertex(vertexIndex++, v[0], v[1], v[2]);
  }

  const std::vector<long>& triangles = shapeInfo.triangles;
  const int numTriangles = triangles.size() / 3;
  for(int j=0; j < numTriangles; ++j){
    int t0 = triangles[j*3] + vertexIndexBase;
    int t1 = triangles[j*3+1] + vertexIndexBase;
    int t2 = triangles[j*3+2] + vertexIndexBase;
    coldetModel->setTriangle( triangleIndex++, t0, t1, t2);
  }

}

void ModelLoaderHelper::addLinkVerticesAndTriangles(ColdetModelPtr& coldetModel, const LinkInfo& linkInfo)
{
  int vertexIndex = 0;
  int triangleIndex = 0;

  const TransformedShapeIndexSequence& shapeIndices = linkInfo.shapeIndices;

  Matrix44 E(Matrix44::Identity());
  for(unsigned int i=0; i < shapeIndices.size(); i++){
    addLinkVerticesAndTriangles(coldetModel, shapeIndices[i], E, shapeInfoSeq_,
                                vertexIndex, triangleIndex);
  }

  Matrix44 T(Matrix44::Identity());
  const std::vector<SensorInfo>& sensors = linkInfo.sensors;
  for (unsigned int i=0; i<sensors.size(); i++){
    const SensorInfo& sensor = sensors[i];
    calcRodrigues(T, Vector3(sensor.rotation[0], sensor.rotation[1],
                             sensor.rotation[2]), sensor.rotation[3]);
    T(0,3) = sensor.translation[0];
    T(1,3) = sensor.translation[1];
    T(2,3) = sensor.translation[2];
    const TransformedShapeIndexSequence& shapeIndices = sensor.shapeIndices;
    for (unsigned int j=0; j<shapeIndices.size(); j++){
      addLinkVerticesAndTriangles(coldetModel, shapeIndices[j], T,
                                  shapeInfoSeq_,
                                  vertexIndex, triangleIndex);
    }
  }
}

void ModelLoaderHelper::addLinkPrimitiveInfo(ColdetModelPtr& coldetModel,
                                             const double *R, const double *p,
                                             const ShapeInfo& shapeInfo)
{
  switch(shapeInfo.primitiveType){
  case SP_BOX:
    coldetModel->setPrimitiveType(ColdetModel::SP_BOX);
    break;
  case SP_CYLINDER:
    coldetModel->setPrimitiveType(ColdetModel::SP_CYLINDER);
    break;
  case SP_CONE:
    coldetModel->setPrimitiveType(ColdetModel::SP_CONE);
    break;
  case SP_SPHERE:
    coldetModel->setPrimitiveType(ColdetModel::SP_SPHERE);
    break;
  case SP_PLANE:
    coldetModel->setPrimitiveType(ColdetModel::SP_PLANE);
    break;
  default:
    break;
  }
  coldetModel->setNumPrimitiveParams(shapeInfo.primitiveParameters.size());
  for (unsigned int i=0; i<shapeInfo.primitiveParameters.size(); i++){
    coldetModel->setPrimitiveParam(i, shapeInfo.primitiveParameters[i]);
  }
  coldetModel->setPrimitivePosition(R, p);
}

void ModelLoaderHelper::setExtraJoints()
{
  body_->extraJoints.clear();
  int n = extraJointInfoSeq_.size();

  for(int i=0; i < n; ++i){
		const ExtraJointInfo& extraJointInfo = extraJointInfoSeq_[i];
    Body::ExtraJoint joint;
		joint.name = extraJointInfo.name;
		joint.link[0] = body_->link(string(extraJointInfo.link[0]));
    joint.link[1] = body_->link(string(extraJointInfo.link[1]));

    Body::ExtraJointType jointType = extraJointInfo.jointType;
    if(jointType == Body::EJ_XY){
      joint.type = Body::EJ_XY;
    }else if(jointType == Body::EJ_XYZ){
      joint.type = Body::EJ_XYZ;
    }else if(jointType == Body::EJ_Z){
      joint.type = Body::EJ_Z;
		}

		joint.axis = Vector3(extraJointInfo.axis[0], extraJointInfo.axis[1], extraJointInfo.axis[2] );
		joint.point[0] = Vector3(extraJointInfo.point[0][0], extraJointInfo.point[0][1], extraJointInfo.point[0][2] );
		joint.point[1] = Vector3(extraJointInfo.point[1][0], extraJointInfo.point[1][1], extraJointInfo.point[1][2] );

    body_->extraJoints.push_back(joint);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////

namespace hrp {

HRPMODEL_API bool loadBodyFromBodyInfo(BodyPtr body, BodyInfo_ptr bodyInfo, bool loadGeometryForCollisionDetection = false, Link *(*f)()=NULL)
  throw (ModelLoaderException)
{
    if(bodyInfo){
        ModelLoaderHelper helper;
        if (f) helper.setLinkFactory(f);
        if(loadGeometryForCollisionDetection){
            helper.enableCollisionDetectionModelLoading(true);
        }
        return helper.createBody(body, bodyInfo);
    }
    return false;
};

}; // namespace hrp


bool hrp::loadBodyFromModelLoader(BodyPtr body, const char* url,  bool loadGeometryForCollisionDetection)
{
  ModelLoader ml;
  BodyInfo_ptr bodyInfo = ml.loadBodyInfo(url);

  if(bodyInfo!=NULL){
    ModelLoaderHelper helper;
    if(loadGeometryForCollisionDetection){
      helper.enableCollisionDetectionModelLoading(true);
    }
    return helper.createBody(body, bodyInfo);
  }

  return false;
}
