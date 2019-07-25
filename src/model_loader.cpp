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
   @author Ergovision
   @author kdaic
*/

#include "model_loader.hpp"

#include "Link.h"
#include "Sensor.h"
#include "Light.h"
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/ColdetModel.h>
#include <stack>

using namespace std;
using namespace hrp;

void ModelLoader::setParam(std::string param, int value){
    if(param == "AABBType")
        AABBdataType_ = (OpenHRP::ModelLoader::AABBdataType)value;
    else
        ;
}

void ShapeSetInfo_impl::createAppearanceInfo(){
    appearances_.length(1);
    AppearanceInfo& appInfo = appearances_[0];
    appInfo.normalPerVertex = false;
    appInfo.colorPerVertex = false;
    appInfo.solid = false;
    appInfo.creaseAngle = 0.0;
    appInfo.textureIndex = -1;
    appInfo.normals.length(0);
    appInfo.normalIndices.length(0);
    appInfo.colors.length(0);
    appInfo.colorIndices.length(0);
    appInfo.materialIndex = 0;
    appInfo.textureIndex = -1;

    materials_.length(1);
    MaterialInfo& material = materials_[0];
    material.ambientIntensity = 0.2;
    material.shininess = 0.2;
    material.transparency = 0.0;
    for(int j = 0 ; j < 3 ; j++){
        material.diffuseColor[j]  = 0.8;
        material.emissiveColor[j] = 0.0;
        material.specularColor[j] = 0.0;
    }
}

void ModelLoader::changetoBoundingBox(unsigned int* inputData){
    const double EPS = 1.0e-6;
    createAppearanceInfo();
    std::vector<Vector3> boxSizeMap;
    std::vector<Vector3> boundingBoxData;

    for(unsigned int i=0; i<links_.length(); i++){
        int _depth;
        if( AABBdataType_ == OpenHRP::ModelLoader::AABB_NUM )
            _depth = linkColdetModels[i]->numofBBtoDepth(inputData[i]);
        else
            _depth = inputData[i];
        if( _depth >= links_[i].AABBmaxDepth)
            _depth = links_[i].AABBmaxDepth-1;
        if(_depth >= 0 ){
            linkColdetModels[i]->getBoundingBoxData(_depth, boundingBoxData);
            std::vector<TransformedShapeIndex> tsiMap;
            links_[i].shapeIndices.length(0);
            std::vector<SensorInfo>& sensors = links_[i].sensors;
            for (unsigned int j=0; j<sensors.length(); j++){
                SensorInfo& sensor = sensors[j];
                sensor.shapeIndices.length(0);
            }

            for(unsigned int j=0; j<boundingBoxData.size()/2; j++){

                bool flg=false;
                unsigned int k=0;
                for( ; k<boxSizeMap.size(); k++)
                    if((boxSizeMap[k] - boundingBoxData[j*2+1]).norm() < EPS)
                        break;
                if( k<boxSizeMap.size() )
                    flg=true;
                else{
                    boxSizeMap.push_back(boundingBoxData[j*2+1]);
                    setBoundingBoxData(boundingBoxData[j*2+1],k);
                }

                if(flg){
                    unsigned int l=0;
                    for( ; l<tsiMap.size(); l++){
                        Vector3 p(tsiMap[l].transformMatrix[3],tsiMap[l].transformMatrix[7],tsiMap[l].transformMatrix[11]);
                        if((p - boundingBoxData[j*2]).norm() < EPS && tsiMap[l].shapeIndex == (int)k)
                            break;
                    }
                    if( l==tsiMap.size() )
                        flg=false;
                }

                if(!flg){
                    int num = links_[i].shapeIndices.length();
                    links_[i].shapeIndices.length(num+1);
                    TransformedShapeIndex& tsi = links_[i].shapeIndices[num];
                    tsi.inlinedShapeTransformMatrixIndex = -1;
                    tsi.shapeIndex = k;
                    Matrix44 T(Matrix44::Identity());
                    for(int p = 0,row=0; row < 3; ++row)
                       for(int col=0; col < 4; ++col)
                            if(col==3){
                                switch(row){
                                    case 0:
                                        tsi.transformMatrix[p++] = boundingBoxData[j*2][0];
                                        break;
                                     case 1:
                                        tsi.transformMatrix[p++] = boundingBoxData[j*2][1];
                                        break;
                                     case 2:
                                        tsi.transformMatrix[p++] = boundingBoxData[j*2][2];
                                        break;
                                     default:
                                        ;
                                }
                            }else
                                tsi.transformMatrix[p++] = T(row, col);

                    tsiMap.push_back(tsi);
                }
            }
        }
        linkShapeIndices_[i] = links_[i].shapeIndices;
    }
}



BodyInfo_ptr ModelLoader::getBodyInfoEx(const char* url0, const ModelLoadOption& option)
    throw (SystemException, ModelLoaderException)
{
    string url(url0);

    BodyInfo_ptr bodyInfo = 0;
    string filename(url);
    struct stat statbuff;
    time_t mtime = 0;

    // get a file modification time
    if( stat( filename.c_str(), &statbuff ) == 0 ){
        mtime = statbuff.st_mtime;
    }

    UrlToBodyInfoMap::iterator p = urlToBodyInfoMap.find(url);
    if(p != urlToBodyInfoMap.end() && mtime == getLastUpdateTime(p->second) && checkInlineFileUpdateTime(p->second)){
        bodyInfo = p->second->_this();
        cout << string("cache found for ") + url << endl;
        if(option.AABBdata.length()){
            setParam(p->second,"AABBType", (int)option.AABBtype);
            int length=option.AABBdata.length();
            unsigned int* _AABBdata = new unsigned int[length];
            for(int i=0; i<length; i++)
                _AABBdata[i] = option.AABBdata[i];
            changetoBoundingBox(p->second,_AABBdata);
            delete[] _AABBdata;
        }
        return bodyInfo;
    }
    return loadBodyInfoEx(url0, option);
}


BodyInfo_ptr ModelLoader::getBodyInfo(const char* url)
    throw (SystemException, ModelLoaderException)
{
  OpenHRP::ModelLoader::ModelLoadOption option;
  option.readImage = false;
  option.AABBdata.length(0);
  option.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
  return getBodyInfoEx(url, option);
}

BodyInfo_ptr ModelLoader::loadBodyInfoFromModelFile(const string url, const ModelLoadOption option)
{
  cout << "loading " << url << endl;
  BodyInfo_ptr bodyInfo;
  try {
    BodyInfo_impl* p = new BodyInfo_impl(poa);
    p->setParam("readImage", option.readImage);
    p->loadModelFile(url);
    bodyInfo = p;
  }
  catch(OpenHRP::ModelLoader::ModelLoaderException& ex){
    cout << "loading failed.\n";
    cout << ex.description << endl;
    throw;
  }

  cout << "The model was successfully loaded ! " << endl;

  urlToBodyInfoMap[url] = bodyInfo;

  string filename( url );
  struct stat statbuff;
  time_t mtime = 0;

  // get a file modification time
  if( stat( filename.c_str(), &statbuff ) == 0 ){
    mtime = statbuff.st_mtime;
  }
  setLastUpdateTime(bodyInfo, mtime );

  return bodyInfo;
}

/*!
  @if jp
  @brief モデルファイルをロードし、BodyInfoを構築する。
  @else
  @brief This function loads a model file and creates a BodyInfo object.
  @param url The url to a model file
  @endif
*/
void ModelLoader::loadModelFile(const std::string& url)
{
  string filename( url );

  // URL文字列の' \' 区切り子を'/' に置き換え  Windows ファイルパス対応
  string url2;
  url2 = filename;
  replace( url2, string("\\"), string("/") );
  filename = url2;
  url_ = url2;


  ModelNodeSet modelNodeSet;
  modelNodeSet.sigMessage.connect(boost::bind(&putMessage, _1));

  bool result = false;

  try	{
    result = modelNodeSet.loadModelFile( filename );

    if(result){
      applyTriangleMeshShaper(modelNodeSet.humanoidNode());
    }
    cout.flush();

  } catch(const ModelNodeSet::Exception& ex) {
    cout << ex.what() << endl;
    cout << "Retrying to load the file as a standard VRML file" << endl;
    try {
      VrmlParser parser;
      parser.load(filename);

      links_.length(1);
      LinkInfo &linfo = links_[0];
      linfo.name = CORBA::string_dup("root");
      linfo.parentIndex = -1;
      linfo.jointId = -1;
      linfo.jointType = CORBA::string_dup("fixed");
      linfo.jointValue = 0;
      for (int i=0; i<3; i++){
        linfo.jointAxis[i] = 0;
        linfo.translation[i] = 0;
        linfo.rotation[i] = 0;
        linfo.centerOfMass[i] = 0;
      }
      linfo.jointAxis[2] = 1;
      linfo.rotation[2] = 1; linfo.rotation[3] = 0;
	    linfo.mass = 0;
	    for (int i=0; i<9; i++) linfo.inertia[i] = 0;


      Matrix44 E(Matrix44::Identity());

      while(VrmlNodePtr node = parser.readNode()){
        if(!node->isCategoryOf(PROTO_DEF_NODE)){
          applyTriangleMeshShaper(node);
          traverseShapeNodes(node.get(), E, linfo.shapeIndices, linfo.inlinedShapeTransformMatrices, &topUrl());
        }
      }
      return;
    } catch(EasyScanner::Exception& ex){
      cout << ex.getFullMessage() << endl;
      throw ModelLoader::ModelLoaderException(ex.getFullMessage().c_str());
    }
  }

  if(!result){
    throw ModelLoader::ModelLoaderException("The model file cannot be loaded.");
  }

  const string& humanoidName = modelNodeSet.humanoidNode()->defName;
  name_ = humanoidName.c_str();
  const MFString& info = modelNodeSet.humanoidNode()->fields["info"].mfString();
  info_.length(info.size());
  for (unsigned int i=0; i<info_.length(); i++){
    info_[i] = CORBA::string_dup(info[i].c_str());
  }

  int numJointNodes = modelNodeSet.numJointNodes();

  links_.length(numJointNodes);
  if( 0 < numJointNodes ) {
    int currentIndex = 0;
    JointNodeSetPtr rootJointNodeSet = modelNodeSet.rootJointNodeSet();
    readJointNodeSet(rootJointNodeSet, currentIndex, -1);
  }

  linkShapeIndices_.length(numJointNodes);
  for(int i = 0 ; i < numJointNodes ; ++i) {
    linkShapeIndices_[i] = links_[i].shapeIndices;
  }

  // build coldetModels
  linkColdetModels.resize(numJointNodes);
  for(int linkIndex = 0; linkIndex < numJointNodes ; ++linkIndex){
    ColdetModelPtr coldetModel(new ColdetModel());
    coldetModel->setName(std::string(links_[linkIndex].name));
    int vertexIndex = 0;
    int triangleIndex = 0;

    Matrix44 E(Matrix44::Identity());
    const TransformedShapeIndexSequence& shapeIndices = linkShapeIndices_[linkIndex];
    setColdetModel(coldetModel, shapeIndices, E, vertexIndex, triangleIndex);

    Matrix44 T(Matrix44::Identity());
    const SensorInfoSequence& sensors = links_[linkIndex].sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
      const SensorInfo& sensor = sensors[i];
      calcRodrigues(T, Vector3(sensor.rotation[0], sensor.rotation[1],
                               sensor.rotation[2]), sensor.rotation[3]);
      T(0,3) = sensor.translation[0];
      T(1,3) = sensor.translation[1];
      T(2,3) = sensor.translation[2];
      const TransformedShapeIndexSequence& sensorShapeIndices = sensor.shapeIndices;
      setColdetModel(coldetModel, sensorShapeIndices, T, vertexIndex, triangleIndex);
    }

    if(triangleIndex>0)
      coldetModel->build();

    linkColdetModels[linkIndex] = coldetModel;
    links_[linkIndex].AABBmaxDepth = coldetModel->getAABBTreeDepth();
    links_[linkIndex].AABBmaxNum = coldetModel->getAABBmaxNum();
  }

	int n = modelNodeSet.numExtraJointNodes();
	extraJoints_.length(n);
  for(int i=0; i < n; ++i){

    TProtoFieldMap& f = modelNodeSet.extraJointNode(i)->fields;
    ExtraJointInfo extraJointInfo(new ExtraJointInfo());
		extraJointInfo->name =  CORBA::string_dup( modelNodeSet.extraJointNode(i)->defName.c_str() );

    std::string link1Name, link2Name;
		copyVrmlField( f, "link1Name", link1Name );
		copyVrmlField( f, "link2Name", link2Name );
		extraJointInfo.link[0] = link1Name;
		extraJointInfo.link[1] = link2Name;

		string jointType;
		copyVrmlField( f, "jointType", jointType);
    if(jointType == "xy"){
      extraJointInfo->jointType = EJ_XY;
    } else if(jointType == "xyz"){
      extraJointInfo->jointType = EJ_XYZ;
    } else if(jointType == "z"){
      extraJointInfo->jointType = EJ_Z;
    }else {
      throw ModelNodeSet::Exception(str(format("JointType \"%1%\" is not supported.") % jointType));
    }
    copyVrmlField( f, "jointAxis", extraJointInfo.axis );
		copyVrmlField( f, "link1LocalPos", extraJointInfo.point[0] );
		copyVrmlField( f, "link2LocalPos", extraJointInfo.point[1] );

		extraJoints_[i] = extraJointInfo;
  }
}

BodyInfo_ptr ModelLoader::loadBodyInfo(const char* url)
{
  // BodyInfo_ptr bodyInfo;
  // try {
  //   bodyInfo = getBodyInfo(url);
  // } catch(SystemException& ex) {
  //   std::cerr << "SystemException raised by ModelLoader: " << ex._rep_id() << std::endl;
  // } catch(ModelLoaderException& ex){
  //   std::cerr << "ModelLoaderException : " << ex.description << std::endl;
  // }
  // return bodyInfo;

  ModelLoadOption option;
  option.readImage = false;
  option.AABBdata.length(0);
  option.AABBtype = ModelLoader::AABB_NUM;
  BodyInfo* bodyInfo = loadBodyInfoFromModelFile(std::string(url), option);
  return bodyInfo->_this();
}

BodyInfo_ptr ModelLoader::loadBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option)
    throw (SystemException, ModelLoaderException)
{
    BodyInfo_ptr bodyInfo = loadBodyInfoFromModelFile(std::string(url), option);
    if(option.AABBdata.length()){
        setParam(bodyInfo,"AABBType", (int)option.AABBtype);
        int length=option.AABBdata.length();
        unsigned int* _AABBdata = new unsigned int[length];
        for(int i=0; i<length; i++)
            _AABBdata[i] = option.AABBdata[i];
        changetoBoundingBox(bodyInfo,_AABBdata);
        delete[] _AABBdata;
    }
    return bodyInfo->this();
}


void ModelLoader::createAppearanceInfo() {
  appearances_.length(1);
  AppearanceInfo& appInfo = appearances_[0];
  appInfo.normalPerVertex = false;
  appInfo.colorPerVertex = false;
  appInfo.solid = false;
  appInfo.creaseAngle = 0.0;
  appInfo.textureIndex = -1;
  appInfo.normals.length(0);
  appInfo.normalIndices.length(0);
  appInfo.colors.length(0);
  appInfo.colorIndices.length(0);
  appInfo.materialIndex = 0;
  appInfo.textureIndex = -1;

  materials_.length(1);
  MaterialInfo& material = materials_[0];
  material.ambientIntensity = 0.2;
  material.shininess = 0.2;
  material.transparency = 0.0;
  for(int j = 0 ; j < 3 ; j++){
    material.diffuseColor[j]  = 0.8;
    material.emissiveColor[j] = 0.0;
    material.specularColor[j] = 0.0;
  }
}

void ModelLoader::changetoBoundingBox(unsigned int* inputData){
  const double EPS = 1.0e-6;
  createAppearanceInfo();
  std::vector<Vector3> boxSizeMap;
  std::vector<Vector3> boundingBoxData;

  for(unsigned int i=0; i<links_.length(); i++){
    int _depth;
    if( AABBdataType_ == ModelLoader::AABB_NUM )
      _depth = linkColdetModels[i]->numofBBtoDepth(inputData[i]);
    else
      _depth = inputData[i];
    if( _depth >= links_[i].AABBmaxDepth)
      _depth = links_[i].AABBmaxDepth-1;
    if(_depth >= 0 ){
      linkColdetModels[i]->getBoundingBoxData(_depth, boundingBoxData);
      std::vector<TransformedShapeIndex> tsiMap;
      links_[i].shapeIndices.length(0);
      std::vector<SensorInfo>& sensors = links_[i].sensors;
      for (unsigned int j=0; j<sensors.length(); j++){
        SensorInfo& sensor = sensors[j];
        sensor.shapeIndices.length(0);
      }

      for(unsigned int j=0; j<boundingBoxData.size()/2; j++){

        bool flg=false;
        unsigned int k=0;
        for( ; k<boxSizeMap.size(); k++)
          if((boxSizeMap[k] - boundingBoxData[j*2+1]).norm() < EPS)
            break;
        if( k<boxSizeMap.size() )
          flg=true;
        else{
          boxSizeMap.push_back(boundingBoxData[j*2+1]);
          setBoundingBoxData(boundingBoxData[j*2+1],k);
        }

        if(flg){
          unsigned int l=0;
          for( ; l<tsiMap.size(); l++){
            Vector3 p(tsiMap[l].transformMatrix[3],tsiMap[l].transformMatrix[7],tsiMap[l].transformMatrix[11]);
            if((p - boundingBoxData[j*2]).norm() < EPS && tsiMap[l].shapeIndex == (int)k)
              break;
          }
          if( l==tsiMap.size() )
            flg=false;
        }

        if(!flg){
          int num = links_[i].shapeIndices.length();
          links_[i].shapeIndices.length(num+1);
          TransformedShapeIndex& tsi = links_[i].shapeIndices[num];
          tsi.inlinedShapeTransformMatrixIndex = -1;
          tsi.shapeIndex = k;
          Matrix44 T(Matrix44::Identity());
          for(int p = 0,row=0; row < 3; ++row)
            for(int col=0; col < 4; ++col)
              if(col==3){
                switch(row){
                case 0:
                  tsi.transformMatrix[p++] = boundingBoxData[j*2][0];
                  break;
                case 1:
                  tsi.transformMatrix[p++] = boundingBoxData[j*2][1];
                  break;
                case 2:
                  tsi.transformMatrix[p++] = boundingBoxData[j*2][2];
                  break;
                default:
                  ;
                }
              }else
                tsi.transformMatrix[p++] = T(row, col);

          tsiMap.push_back(tsi);
        }
      }
    }
    linkShapeIndices_[i] = links_[i].shapeIndices;
  }
}



BodyInfo_ptr ModelLoader::getBodyInfoEx(const char* url0, const OpenHRP::ModelLoader::ModelLoadOption& option)
    throw (SystemException, ModelLoaderException)
{
    string url(url0);

    BodyInfo_ptr bodyInfo = 0;
    string filename(url);
    struct stat statbuff;
    time_t mtime = 0;

    // get a file modification time
    if( stat( filename.c_str(), &statbuff ) == 0 ){
        mtime = statbuff.st_mtime;
    }

    UrlToBodyInfoMap::iterator p = urlToBodyInfoMap.find(url);
    if(p != urlToBodyInfoMap.end() && mtime == getLastUpdateTime(p->second) && checkInlineFileUpdateTime(p->second)){
        bodyInfo = p->second->_this();
        cout << string("cache found for ") + url << endl;
        if(option.AABBdata.length()){
            setParam(p->second,"AABBType", (int)option.AABBtype);
            int length=option.AABBdata.length();
            unsigned int* _AABBdata = new unsigned int[length];
            for(int i=0; i<length; i++)
                _AABBdata[i] = option.AABBdata[i];
            changetoBoundingBox(p->second,_AABBdata);
            delete[] _AABBdata;
        }
        return bodyInfo;
    }
    return loadBodyInfoEx(url0, option);
}

BodyInfo_ptr ModelLoader::getBodyInfo(const char* url)
    throw (CORBA::SystemException, OpenHRP::ModelLoader::ModelLoaderException)
{
    OpenHRP::ModelLoader::ModelLoadOption option;
    option.readImage = false;
    option.AABBdata.length(0);
    option.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
    return getBodyInfoEx(url, option);
}




////////////////////////////////////////////////////////////////////////////////////////////


bool ModelLoaderHelper::createBody(BodyPtr& body, BodyInfo_ptr bodyInfo)
{
  this->body = body;

  const char* name = bodyInfo->name();
  body->setModelName(name);
  body->setName(name);

  int n = bodyInfo->links()->length();
  links_ = bodyInfo->links();
  shapes_ = bodyInfo->shapes();
	extraJoints_ = bodyInfo->extraJoints();

  int rootIndex = -1;

  for(int i=0; i < n; ++i){
    if(links_[i].parentIndex < 0){
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
    body->setRootLink(rootLink);
    body->setDefaultRootPosition(rootLink->b, rootLink->Rs);

    body->installCustomizer();
    body->initializeConfiguration();

		setExtraJoints();

    return true;
  }

  return false;
}


bool ModelLoader::loadBodyFromModelLoader(BodyPtr body, const char* url, bool loadGeometryForCollisionDetection)
{
    BodyInfo_ptr bodyInfo = loadBodyInfo(url);

    if(!CORBA::is_nil(bodyInfo)){
        ModelLoaderHelper helper;
        if(loadGeometryForCollisionDetection){
            helper.enableCollisionDetectionModelLoading(true);
        }
        return helper.createBody(body, bodyInfo);
    }

    return false;
}
