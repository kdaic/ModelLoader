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

#include "body_info.hpp"
#include <hrpUtil/ImageConverter.h>
#include <hrpUtil/VrmlParser.h>
#include <sys/stat.h>

using namespace hrp;

/////////////////////////////////////////////////////////////////////////////////////////////

class SceneInfo : public virtual ShapeSetInfo
{
public:

  SceneInfo();
  virtual ~SceneInfo();

  virtual std::string url();
  virtual TransformedShapeIndexSequence* shapeIndices();

  void load(const std::string& filename);

protected:

  virtual const std::string& topUrl();

private:

  /// 形状ファイルのURL。
  std::string url_;
  /// LinkInfo の shapeIndices と同じ。
  TransformedShapeIndexSequence shapeIndices_;
  std::vector<boost::array<double,12> > inlinedShapeTransformMatrices_;

};

typedef SceneInfo* SceneInfo_ptr;

/////////////////////////////////////////////////////////////////////////////////////////////

/// @brief ModelLoaderインタフェース定義
/// テキスト記述されたモデル情報のファイルを読み込み、
/// データを構造化したBodyInfoオブジェクトとして提供する。
class ModelLoader {

public:
  ModelLoader(){};
  ~ModelLoader(){};

  typedef std::map<std::string, BodyInfo*> UrlToBodyInfoMap;
  UrlToBodyInfoMap urlToBodyInfoMap_;

  /// BodyInfoオブジェクトを得る。
  /// @details
  /// 本メソッドでは、指定されたファイルが以前に読み込まれていれば、
  /// その読み込み内容を再利用することにより、高速に結果を返す。
  /// ただし、指定されたファイルがまだ読み込まれていない場合や、
  /// 以前の読み込みの後にファイルが更新された場合は、
  /// load()メソッドと同じ処理となる。
  /// @param[in] url モデルファイルのURL
  BodyInfo_ptr getBodyInfo(const char* url)
    throw(ModelLoaderException);

  /// オプション付きで、getBodyInfoを実行
  /// @param[in] url モデルファイルのURL
  /// @param[in] option
  BodyInfo_ptr getBodyInfoEx(const char* url, const ModelLoadOption& option )
    throw(ModelLoaderException);


  /// モデルファイルをロードし、BodyInfoオブジェクトを得る。
  /// @param[in] url モデルファイルのURL
  BodyInfo_ptr loadBodyInfo(const char* url)
    throw(ModelLoaderException);

  BodyInfo_ptr loadBodyInfoEx(const char* url, const ModelLoadOption& option)
    throw(ModelLoaderException);

  /// 素のVRMLファイルを読み込み、含まれる形状のデータ一式を SceneInfo として返す。
  SceneInfo_ptr loadSceneInfo(const char* url)
    throw(ModelLoaderException);

  /// 以前に読み込んだファイルの内容（getBody()で利用される）を消去する。
  void clearData();


  BodyInfo_ptr loadBodyInfoFromModelFile(const std::string url, const ModelLoadOption option );
  bool loadBodyFromModelLoader(BodyPtr body, const char* url, bool loadGeometryForCollisionDetection);

};




#endif // MODEL_LOADER_HPP_INCLUDE
