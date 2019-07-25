/*
 * Copyright (c) 2008, kdaic, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/**
   @author Shin'ichiro Nakaoka
   @author kdaic
*/

#ifndef HRPMODEL_MODEL_LOADER_UTIL_H_INCLUDED
#define HRPMODEL_MODEL_LOADER_UTIL_H_INCLUDED

#ifdef __WIN32__
#pragma warning(disable:4996)
#endif

#include "Body.h"
#include "model_loader.hpp"
#include <string>
#include <sstream>

///////////////////////////////////////////////////////////////////////////////////////////

namespace
{

static Link *createNewLink() { return new Link(); }

class ModelLoaderHelper
{
public:
  ModelLoaderHelper() {
    collisionDetectionModelLoading = false;
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

  Link* createLink(int index, const Matrix33& parentRs);
  void createSensors(Link* link, const SensorInfoSequence& sensorInfoSeq, const Matrix33& Rs);
  void createLights(Link* link, const LightInfoSequence& lightInfoSeq, const Matrix33& Rs);
  void createColdetModel(Link* link, const LinkInfo& linkInfo);
  void addLinkPrimitiveInfo(ColdetModelPtr& coldetModel,
                            const double *R, const double *p,
                            const ShapeInfo& shapeInfo);
  void addLinkVerticesAndTriangles(ColdetModelPtr& coldetModel, const LinkInfo& linkInfo);
  void addLinkVerticesAndTriangles(ColdetModelPtr& coldetModel, const TransformedShapeIndex& tsi, const Matrix44& Tparent, ShapeInfoSequence_var& shapes, int& vertexIndex, int& triangleIndex);
  void setExtraJoints();
};

};

///////////////////////////////////////////////////////////////////////////////////////////

namespace hrp
{
    HRPMODEL_API bool loadBodyFromBodyInfo(BodyPtr body, BodyInfo_ptr bodyInfo, bool loadGeometryForCollisionDetection = false, Link *(*f)()=NULL);
    HRPMODEL_API BodyInfo loadBodyInfo(const char* url, int& argc, char* argv[]);
    HRPMODEL_API BodyInfo loadBodyInfo(const char* url, CORBA_ORB_var orb);
    HRPMODEL_API BodyInfo loadBodyInfo(const char* url, CosNaming::NamingContext_var cxt);
    HRPMODEL_API bool loadBodyFromModelLoader(BodyPtr body, const char* url, CORBA_ORB_var orb, bool loadGeometryForCollisionDetection = false);
    HRPMODEL_API bool loadBodyFromModelLoader(BodyPtr body, const char* url, CosNaming::NamingContext_var cxt,  bool loadGeometryForCollisionDetection = false);
    HRPMODEL_API bool loadBodyFromModelLoader(BodyPtr body, const char* url, int& argc, char* argv[],  bool loadGeometryForCollisionDetection = false);
    HRPMODEL_API OpenHRP::ModelLoader_var getModelLoader(CosNaming::NamingContext_var cxt);
    HRPMODEL_API OpenHRP::ModelLoader_var getModelLoader(CORBA_ORB_var orb);
};


#endif
