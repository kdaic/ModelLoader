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


///////////////////////////////////////////////////////////////////////////////////////////



// the dynamic casts are necessary since the changetoBoundingBox functions are not part of BodyInfo class.
static void g_setLastUpdateTime(BodyInfo* bodyInfo, time_t time)
{
    BodyInfo* pBodyInfo = bodyInfo;
    if( !!pBodyInfo ) {
        pBodyInfo->setLastUpdateTime(time);
        return;
    }
    throw ModelLoaderException("setLastUpdateTime invalid pointer");
};

static time_t g_getLastUpdateTime(BodyInfo* bodyInfo) {
    BodyInfo* pBodyInfo = bodyInfo;
    if( !!pBodyInfo ) {
        return pBodyInfo->getLastUpdateTime();
    }
    throw ModelLoaderException("getLastUpdateTime invalid pointer");
}

static bool g_checkInlineFileUpdateTime(BodyInfo* bodyInfo)
{
  BodyInfo* pBodyInfo = bodyInfo;
  if( !!pBodyInfo ) {
    return pBodyInfo_impl->checkInlineFileUpdateTime();
  }
  throw ModelLoaderException("checkInlineFileUpdateTime invalid pointer");
}

static void g_setParam(BodyInfo* bodyInfo, std::string param, int value)
{
  BodyInfo* pBodyInfo = bodyInfo;
  if( !!pBodyInfo ) {
    pBodyInfo->setParam(param,value);
    return;
  }
  throw ModelLoaderException("setParam(param,value) invalid pointer");
}

static void g_changetoBoundingBox(BodyInfo* bodyInfo, unsigned int* depth)
{
  BodyInfo* pBodyInfo = bodyInfo;
  if( !!pBodyInfo ) {
    pBodyInfo->changetoBoundingBox(depth);
    return;
  }
  throw ModelLoaderException("changetoBoundingBox(depth) invalid pointer");
}


////////////////////////////////////////////////////////////////


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

  UrlToBodyInfoMap::iterator p = urlToBodyInfoMap_.find(url);
  if( p != urlToBodyInfoMap_.end()
      && mtime == g_getLastUpdateTime(p->second)
      && g_checkInlineFileUpdateTime(p->second) ) {
    bodyInfo = p->second->_this();
    cout << string("cache found for ") + url << endl;
    if(option.AABBdata.length()){
      g_setParam(p->second,"AABBType", (int)option.AABBtype);
      int length=option.AABBdata.length();
      unsigned int* _AABBdata = new unsigned int[length];
      for(int i=0; i<length; i++)
        _AABBdata[i] = option.AABBdata[i];
      g_changetoBoundingBox(p->second,_AABBdata);
      delete[] _AABBdata;
    }
    return bodyInfo;
  }
  return loadBodyInfoEx(url0, option);
}


BodyInfo_ptr ModelLoader::getBodyInfo(const char* url)
    throw (SystemException, ModelLoaderException)
{
  ModelLoadOption option;
  option.readImage = false;
  option.AABBdata.length(0);
  option.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
  return getBodyInfoEx(url, option);
}


BodyInfo_ptr ModelLoader::loadBodyInfo(const char* url)
{
  BodyInfo_ptr bodyInfo;
  try {
    bodyInfo = getBodyInfo(url);
  } catch(SystemException& ex) {
    std::cerr << "SystemException raised by ModelLoader: " << ex._rep_id() << std::endl;
  } catch(ModelLoaderException& ex){
    std::cerr << "ModelLoaderException : " << ex.description << std::endl;
  }
  return bodyInfo;
}


BodyInfo_ptr ModelLoader::loadBodyInfoEx(const char* url, const OpenHRP::ModelLoader::ModelLoadOption& option)
    throw (SystemException, ModelLoaderException)
{
    BodyInfo_ptr bodyInfo = loadBodyInfoFromModelFile(std::string(url), option);
    if(option.AABBdata.length()){
        g_setParam(bodyInfo,"AABBType", (int)option.AABBtype);
        int length=option.AABBdata.length();
        unsigned int* _AABBdata = new unsigned int[length];
        for(int i=0; i<length; i++)
            _AABBdata[i] = option.AABBdata[i];
        g_hangetoBoundingBox(bodyInfo,_AABBdata);
        delete[] _AABBdata;
    }
    return bodyInfo->this();
}



BodyInfo_ptr ModelLoader::loadBodyInfoFromModelFile(const string url, const ModelLoadOption option)
{
  cout << "loading " << url << endl;
  BodyInfo_ptr bodyInfo;
  try {
    BodyInfo* p = new BodyInfo();
    p->setParam("readImage", option.readImage);
    p->loadModelFile(url);
    bodyInfo = p;
  }
  catch(ModelLoaderException& ex){
    cout << "loading failed.\n";
    cout << ex.description << endl;
    throw;
  }

  cout << "The model was successfully loaded ! " << endl;

  urlToBodyInfoMap_[url] = bodyInfo;

  string filename( url );
  struct stat statbuff;
  time_t mtime = 0;

  // get a file modification time
  if( stat( filename.c_str(), &statbuff ) == 0 ){
    mtime = statbuff.st_mtime;
  }
  bodyInfo->setLastUpdateTime( mtime );

  return bodyInfo;
}


SceneInfo_ptr ModelLoader::loadSceneInfo(const char* url)
  throw (SystemException, ModelLoaderException)
{
  cout << "loading " << url << endl;

  SceneInfo_impl* sceneInfo;
  try {
    SceneInfo_impl* p = new SceneInfo();
    p->load(url);
    sceneInfo = p;
  }
  catch(ModelLoaderException& ex){
    cout << "loading failed.\n";
    cout << ex.description << endl;
    throw;
  }
  cout << url << " was successfully loaded ! " << endl;

  return sceneInfo->_this();
}


void ModelLoader_impl::clearData()
{
    //UrlToBodyInfoMap::iterator p;
    //for(p = urlToBodyInfoMap.begin(); p != urlToBodyInfoMap.end(); ++p){
    //	BodyInfo_impl* bodyInfo = p->second;
    //	PortableServer::ObjectId_var objectId = poa->servant_to_id(bodyInfo);
    //	poa->deactivate_object(objectId);
    //	bodyInfo->_remove_ref();
    //}
    urlToBodyInfoMap_.clear();
}



////////////////////////////////////////////////////////////////////////////////////////////


bool hrp::loadBodyFromModelLoader(BodyPtr body, const char* url, bool loadGeometryForCollisionDetection)
{
  ModelLoader ml();
  BodyInfo_ptr bodyInfo = ml.loadBodyInfo(url);

  if(!CORBA::is_nil(bodyInfo)){
    ModelLoaderHelper helper;
    if(loadGeometryForCollisionDetection){
      helper.enableCollisionDetectionModelLoading(true);
    }
    return helper.createBody(body, bodyInfo);
  }

  return false;
}
