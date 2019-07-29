#include <gtest/gtest.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpModel/Link.h>
#include <hrpModel/LinkTraverse.h>
#include <hrpModel/JointPath.h>

#include "model_loader.hpp"
#include "ModelLoaderUtil.hpp"

#include <boost/intrusive_ptr.hpp>
#include <boost/array.hpp>

using namespace hrp;

class VrmlNode1;
inline void intrusive_ptr_add_ref(VrmlNode1* obj);
inline void intrusive_ptr_release(VrmlNode1* obj);
class VrmlNode1 {
public:
  VrmlNode1()
  {
    refCounter = 0;
  }
  ~VrmlNode1() {}
  int refCounter;
  friend void intrusive_ptr_add_ref(VrmlNode* obj);
  friend void intrusive_ptr_release(VrmlNode* obj);
};
inline void intrusive_ptr_add_ref(VrmlNode1* obj){
  obj->refCounter++;
}
inline void intrusive_ptr_release(VrmlNode1* obj){
  obj->refCounter--;
  if(obj->refCounter <= 0){
    delete obj;
  }
}

//! VRML Proto definition
class VrmlProto1 : public VrmlNode1
{
public:
  std::string protoName;

  VrmlProto1(const std::string& n) : protoName(n)
  {}
};
typedef boost::intrusive_ptr<VrmlProto1> VrmlProtoPtr1;


//! VRML node which is instance of VRML Prototype
class VrmlProtoInstance1 : public VrmlNode1
{
public:
  VrmlProtoPtr1 proto;
  VrmlProtoInstance1(VrmlProtoPtr1 proto0):
    proto(proto0)
  {
  }
};
typedef boost::intrusive_ptr<VrmlProtoInstance1> VrmlProtoInstancePtr1;

// void g_setJointParameters(int linkInfoIndex, VrmlProtoInstancePtr jointNode) {
// }

// int g_readJointNodeSet(BodyInfo_ptr bip, JointNodeSetPtr jointNodeSet, int& currentIndex ) {
//   int index = currentIndex;
//   currentIndex++;
//   size_t numChildren = jointNodeSet->childJointNodeSets.size();
//   for( size_t i = 0 ; i < numChildren ; ++i ){
//     JointNodeSetPtr childJointNodeSet = jointNodeSet->childJointNodeSets[i];
//     int childIndex = g_readJointNodeSet(bip, childJointNodeSet, currentIndex);
//   }
//   VrmlProtoInstancePtr jNode = jointNodeSet->jointNode;
//   g_setJointParameters(index, jNode);
// }

TEST(BodyInfoTest, loadModeFile) {
  std::string model_url = "./test/model/sample.wrl";
  ModelLoader ml;
  BodyInfo_ptr bip = ml.loadBodyInfo( model_url.c_str() );

  bip->loadModelFile( model_url );

  // ModelNodeSet modelNodeSet;
  // bool result = false;
  // result = modelNodeSet.loadModelFile( model_url );
  // int numJointNodes = modelNodeSet.numJointNodes();
  // if( 0 < numJointNodes ) {
  //   int currentIndex = 0;
  //   JointNodeSetPtr rootJointNodeSet = modelNodeSet.rootJointNodeSet();
  //   g_readJointNodeSet( bip, rootJointNodeSet, currentIndex );
  // }
}

TEST(ModelLoaderTest, loadModelFile) {

  BodyPtr body(new Body());
  // std::string model_url = "./test/model/PA10/pa10.main.wrl";
  std::string model_url = "./test/model/sample.wrl";

  EXPECT_TRUE(loadBodyFromModelLoader(body, model_url.c_str(), false));

  body->calcForwardKinematics();
}
