/*
 * Copyright (c) 2008, kdaic AIST, the University of Tokyo and General Robotix Inc.
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

#ifndef MODEL_LOADER_HPP_INCLUDE
#define MODEL_LOADER_HPP_INCLUDE

#include <cstdlib>
#include <string>
#include <iostream>

#include "Body.h"
#include "Light.h"

typedef std::map<std::string, POA_OpenHRP::BodyInfo*> UrlToBodyInfoMap;
UrlToBodyInfoMap urlToBodyInfoMap;

struct TransformedShapeIndex {

  double transformMatrix[12];

  short inlinedShapeTransformMatrixIndex;

  short shapeIndex;
};

typedef std::vector<TransformedShapeIndex> TransformedShapeIndexSequence;
typedef std::vector<TransformedShapeIndexSequence> AllLinkShapeIndexSequence;

struct HwcInfo {
  std::string_member name;

  long id;

  double translation[3];

  double rotation[4];

  std::string url;

  TransformedShapeIndexSequence shapeIndices;

  std::vector<boost::array<double,12> > inlinedShapeTransformMatrices;
};

struct SegmentInfo {

  std::string_member name;

  double mass;

  double centerOfMass[3];

  double inertia[9];

  double transformMatrix[12];

  TransformedShapeIndexSequence shapeIndices;
};

struct LightInfo {

  std::string name;

  LightType type;

  double transformMatrix[12];

  double ambientIntensity;

  double attenuation[3];

  double color[3];

  double intensity;

  double location[3];

  bool on;

  double radius;

  double direction[3];

  double beamWidth;

  double cutOffAngle;
};

struct LinkInfo {
  std::string name;

  short jointId;

  std::string_member jointType;

  double jointValue;

  double jointAxis[3];

  std::vector<double> ulimit;

  std::vector<double> llimit;

  std::vector<double> uvlimit;

  std::vector<double> lvlimit;

  std::vector<double> climit;

  double translation[3];

  double rotation[4];

  double mass;

  double centerOfMass[3];

  double inertia[9];

  double rotorInertia;

  double rotorResistor;

  double gearRatio;

  double torqueConst;

  double encoderPulse;

  short parentIndex;

  std::vector<short> childIndices;

  std::vector<TransformedShapeIndex> shapeIndices;

  short AABBmaxDepth;

  short AABBmaxNum;

  std::vector<boost::array<double,12> > inlinedShapeTransformMatrices;

  std::vector<SensorInfo> sensors;

  std::vector<HwcInfo> hwcs;

  std::vector<SegmentInfo> segments;

  std::vector<LightInfo> lights;

  std::vector<std::string> specFiles;
};


struct ShapeInfo {

  std::string url;

  ShapePrimitiveType primitiveType;

  std::vector<float> primitiveParameters;

  std::vector<float> vertices;

  std::vecotr<long> triangles;

  long appearanceIndex;
};


struct AppearanceInfo {

  long materialIndex;

  std::vector<float> normals;

  std::vector<long> normalIndices;

  bool normalPerVertex;

  bool solid;

  float creaseAngle;

  std::vector<float> colors;

  std::vector<long> colorIndices;

  bool colorPerVertex;

  long textureIndex;

  std::vector<float> textureCoordinate;

  std::vector<long> textureCoordIndices;

  double textransformMatrix[9];
};

struct MaterialInfo {

  float ambientIntensity;

  float diffuseColor[3];

  float emissiveColor[3];

  float shininess;

  float specularColor[3];

  float transparency;
};

struct TextureInfo {
  std::vector<unsigned char> image;

  short numComponents;

  short width;

  short height;

  bool repeatS;

  bool repeatT;

  std::string url;

  void operator<<= (cdrStream &);
};

struct ExtraJointInfo {
  std::string_member name;

  ExtraJointType jointType;

  double axis[3];

  std::string link[2];

  double point[2];
};

struct ModelLoadOption {

  ::CORBA::Boolean readImage;

  std::vector<short> AABBdata;

  AABBdataType AABBtype;
};

enum AABBdataType { AABB_DEPTH, AABB_NUM /*, __max_AABBdataType=0xffffffff */ };

///////////////////////////////////////////////////////////////////////////////////////////

class ModelLoaderHelper {
public:
  ModelLoaderHelper();
  ~ModelLoaderHelper();
  bool createBody(BodyPtr& body, BodyInfo_ptr bodyInfo);
}

///////////////////////////////////////////////////////////////////////////////////////////

class ModelLoader {
public:
  ModelLoader();
  ~ModelLoader();
  void setParam(std::string param, int value);
  void changetoBoundingBox(unsigned int* inputData);
  BodyInfo_ptr getBodyInfoEx(const char* url0, const ModelLoadOption& option);
  BodyInfo_ptr getBodyInfo(const char* url);
  BodyInfo_ptr loadBodyInfo(const char* url);
  BodyInfo_ptr loadBodyInfoFromModelFile(const string url, const OpenHRP::ModelLoader::ModelLoadOption option);
  void loadModelFile(const std::string& url);
  BodyInfo_ptr loadBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option);
  void createAppearanceInfo();
  void changetoBoundingBox(unsigned int* inputData);
  BodyInfo_ptr getBodyInfoEx(const char* url0, const OpenHRP::ModelLoader::ModelLoadOption& option)
  BodyInfo_ptr getBodyInfo(const char* url);

  bool loadBodyFromModelLoader(BodyPtr body, const char* url, bool loadGeometryForCollisionDetection);

  std::vector<ShapeInfo> shapes_;
  std::vector<AppearanceInfo> appearances_;
  std::vector<MaterialInfo> materials_;
  std::vector<TextureInfo> textures_;

  std::vector<ShapeInfo>  originShapes_;
  std::vector<AppearanceInfo> originAppearances_;
  std::vector<MaterialInfo> originMaterials_;

  TriangleMeshShaper triangleMeshShaper;

  typedef std::map<VrmlShapePtr, int> ShapeNodeToShapeInfoIndexMap;
  ShapeNodeToShapeInfoIndexMap shapeInfoIndexMap;

  std::map<std::string, time_t> fileTimeMap;

private:
  time_t lastUpdate_;
  bool readImage_;
  AABBdataType AABBdataType_;

  std::string name_;
  std::string url_;
  std::vector<std::string> info_;
  std::vector<LinkInfo> links_;

  std::vector<AllLinkShapeIndex> linkShapeIndices_;
  std::vecotr<AllLinkShapeIndex> originlinkShapeIndices_;
  std::vector<ExtraJointInfo> extraJoints_;

  std::vector<ColdetModelPtr> linkColdetModels;
};




#endif // MODEL_LOADER_HPP_INCLUDE
