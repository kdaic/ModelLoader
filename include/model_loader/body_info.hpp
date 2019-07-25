/*
 * Copyright (c) 2008, kdaic, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/*!
  @file body_info.h
  @author Shin'ichiro Nakaoka
  @author kdaic
*/

#ifndef MODEL_LOADER_BODYINFO_HPP_INCLUDED
#define MODEL_LOADER_BODYINFO_HPP_INCLUDED

#include <string>
#include <hrpModel/ModelNodeSet.h>
#include <hrpCollision/ColdetModel.h>

#include "shape_set_info_impl.hpp"

/// 物体モデル情報。
/// 旧IDLのCharacterInfoに対応。
class BodyInfo : ShapeSetInfo
{
public:
  BodyInfo();
  virtual ~BodyInfo();

  char* name();
  char* url();
  StringSequence* info();
  LinkInfoSequence* links();
  AllLinkShapeIndexSequence* linkShapeIndices();
  ExtraJointInfoSequence* extraJoints();

  void loadModelFile(const std::string& filename);

  void setLastUpdateTime(time_t time) { lastUpdate_ = time;};
  time_t getLastUpdateTime() { return lastUpdate_; }
  bool checkInlineFileUpdateTime() { return checkFileUpdateTime(); }

  bool getParam(std::string param);
  void setParam(std::string param, bool value);
  void setParam(std::string param, int value);
  void changetoBoundingBox(unsigned int* depth) ;
  void changetoOriginData();

protected:

  virtual const std::string& topUrl();

private:

  time_t lastUpdate_;
  bool readImage_;
  AABBdataType AABBdataType_;

 ///モデル名
  std:: string name_;

  /// モデルファイルのURL。
  std::string url_;

  /// Humanoidノードにおけるinfoフィールドに記述されたテキスト。
  std::vector<std::string> info_;

  /// リンクの機構情報を全リンクについて格納したデータ。
  /// @details
  /// 本シーケンスにおけるLinkInfoの並びは、
  /// linkIndex(モデルファイルにおけるJointNode出現順。jointIdとは異なる。)
  /// の順とすること。
  std::vector<LinkInfo> links_;

  /// リンクに対応するShapeInfoのインデックス配列を、全てのリンクに関して格納した配列。
  /// @details
  /// CollisionDetector のように、LinkInfoの詳しい情報は必要ないが、
  /// 各リンクのShapeInfoは必要という場合にこのデータを利用する。
  ///
  /// なお、CollisionDetectorを新IDLに対応させる際には、本インタフェースのlinksメンバは
  /// 使わずに済むように改変すること。
  ///
  /// リンクの並びはlinkIndexの順とする。
  AllLinkShapeIndexSequence linkShapeIndices_;

  AllLinkShapeIndexSequence originlinkShapeIndices_;

  /// 閉リンク機構のリンク接続の全情報
  std::vector<ExtraJointInfo> extraJoints_;

  std::vector<ColdetModelPtr> linkColdetModels_;

  int readJointNodeSet(JointNodeSetPtr jointNodeSet, int& currentIndex, int motherIndex);
  void setJointParameters(int linkInfoIndex, VrmlProtoInstancePtr jointNode );
  void setSegmentParameters(int linkInfoIndex, JointNodeSetPtr jointNodeSet);
  void setSensors(int linkInfoIndex, JointNodeSetPtr jointNodeSet);
  void setHwcs(int linkInfoIndex, JointNodeSetPtr jointNodeSet);
  void setLights(int linkInfoIndex, JointNodeSetPtr jointNodeSet);
  void readSensorNode(int linkInfoIndex, SensorInfo& sensorInfo, VrmlProtoInstancePtr sensorNode);
  void readHwcNode(int linkInfoIndex, HwcInfo& hwcInfo, VrmlProtoInstancePtr hwcNode);
  void readLightNode(int linkInfoIndex, LightInfo& LightInfo,
                     std::pair<Matrix44, VrmlNodePtr> &transformedLight);

};


typedef std::map<std::string, BodyInfo*> UrlToBodyInfoMap;
UrlToBodyInfoMap urlToBodyInfoMap;

#endif // MODEL_LOADER_BODYINFO_HPP_INCLUDED
