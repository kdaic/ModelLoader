/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef VRML_UTIL_H_INCLUDED
#define VRML_UTIL_H_INCLUDED

#include <string>
#include <hrpUtil/VrmlNodes.h>
#include <hrpUtil/UrlUtil.h>

using namespace hrp;


/// モデルローダの処理においてエラーが生じたときに生成される例外。
class ModelLoaderException : public std::exception
{
public:
  ModelLoaderException(const std::string& ex="") :
    std::exception() {};

  ~ModelLoaderException() throw() {}
  /// エラーの説明
  std::string  description;
};

void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, std::string& out_s);
void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, std::vector<double>& out_v);
void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, double* out_v);
void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, long* out_v);

//void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, DblSequence3& out_v);
//void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, DblSequence9& out_m);

void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, boost::array<double,3>& out_v);
void copyVrmlField(TProtoFieldMap& fmap, const std::string& name, boost::array<double,9>& out_m);

void copyVrmlRotationFieldToDblArray9(TProtoFieldMap& fieldMap, const std::string name, double* out_R);
void copyVrmlRotationFieldToDblArray4(TProtoFieldMap& fieldMap, const std::string name, double* out_R);

std::string setTexturefileUrl( std::string modelfileDir, hrp::MFString urls);
#endif
