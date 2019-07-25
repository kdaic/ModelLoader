/*
 * Copyright (c) 2008, kdaic, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/*!
  @file ShapeSetInfo_impl.h
  @author Shin'ichiro Nakaoka
  @author kdaic
*/

#ifndef SHAPE_SET_INFO_HPP_INCLUDED
#define SHAPE_SET_INFO_HPP_INCLUDED

#include <string>
#include <hrpUtil/TriangleMeshShaper.h>
#include <hrpUtil/VrmlNodes.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/ColdetModel.h>

#include "model_info.hpp"

/// 形状データ一式を格納するオブジェクト。
class ShapeSetInfo
{

public:

  ShapeSetInfo();
  virtual ~ShapeSetInfo();

  virtual ShapeInfoSequence* shapes();
  virtual AppearanceInfoSequence* appearances();
  virtual MaterialInfoSequence* materials();
  virtual TextureInfoSequence* textures();

protected:

  void applyTriangleMeshShaper(VrmlNodePtr node);
  static void putMessage(const std::string& message);
  std::string& replace(std::string& str, const std::string& sb, const std::string& sa);
  void traverseShapeNodes(VrmlNode* node, const Matrix44& T, TransformedShapeIndexSequence& io_shapeIndices, DblArray12Sequence& inlinedShapeM, const SFString* url = NULL);
  virtual const std::string& topUrl() = 0;
  void setColdetModel(ColdetModelPtr& coldetModel, TransformedShapeIndexSequence shapeIndices, const Matrix44& Tparent, int& vertexIndex, int& triangleIndex);
  void saveOriginalData();
  void restoreOriginalData();
  void createAppearanceInfo();
  void setBoundingBoxData(const Vector3& boxSize, int shapeIndex);
  bool checkFileUpdateTime();
  bool readImage;

  /// 表面の形状と見えの情報を格納するShapeInfoのシーケンス。
  /// LinkInfoにおいて、Linkに対応する情報が本シーケンスのインデックスとして指定される。
  std::vector<ShapeInfo> shapes_;

  /// Appearance情報のシーケンス。
  /// ShapeInfoにおいて、本シーケンスのインデックスが指定される。
  std::vector<AppearanceInfo> appearances_;

  /// Material情報のシーケンス。
  /// AppearanceInfoにおいて、本シーケンスのインデックスが指定される。
  std::vector<MaterialInfo> materials_;

  /// Texture情報のシーケンス。
  /// AppearanceInfoにおいて、本シーケンスのインデックスが指定される。
  std::vector<TextureInfo> textures_;

private:

  std::vector<ShapeInfo>  originShapes_;
  std::vector<AppearanceInfo> originAppearances_;
  std::vector<MaterialInfo> originMaterials_;

  TriangleMeshShaper triangleMeshShaper_;

  typedef std::map<VrmlShapePtr, int> ShapeNodeToShapeInfoIndexMap;
  ShapeNodeToShapeInfoIndexMap shapeInfoIndexMap_;

  std::map<std::string, time_t> fileTimeMap_;

  int createShapeInfo(VrmlShape* shapeNode, const SFString* url);
  void setTriangleMesh(ShapeInfo& shapeInfo, VrmlIndexedFaceSet* triangleMesh);
  void setPrimitiveProperties(ShapeInfo& shapeInfo, VrmlShape* shapeNode);
  int createAppearanceInfo(ShapeInfo& shapeInfo, VrmlShape* shapeNode, VrmlIndexedFaceSet* faceSet, const SFString *url);
  void setColors(AppearanceInfo& appInfo, VrmlIndexedFaceSet* triangleMesh);
  void setNormals(AppearanceInfo& appInfo, VrmlIndexedFaceSet* triangleMesh);
  void setTexCoords(AppearanceInfo& appInfo, VrmlIndexedFaceSet* triangleMesh);
  int createMaterialInfo(VrmlMaterialPtr& materialNode);
  int createTextureInfo(VrmlTexturePtr& textureNode, const SFString *url);
  void createTextureTransformMatrix(AppearanceInfo& appInfo, VrmlTextureTransformPtr& textureTransform );
  std::string getModelFileDirPath(const std::string& url);
  void setColdetModelTriangles(ColdetModelPtr& coldetModel, const TransformedShapeIndex& tsi, const Matrix44& Tparent, int& vertexIndex, int& triangleIndex);

  friend class BodyInfo;
  friend class SceneInfo;
};


#endif // SHAPE_SET_INFO_HPP_INCLUDED
