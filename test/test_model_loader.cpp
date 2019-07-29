#include <gtest/gtest.h>

#include "model_loader.hpp"
#include "ModelLoaderUtil.hpp"

using namespace hrp;

TEST(BodyInfoTest, loadModeFile) {
  std::string model_url = "./test/model/sample.wrl";
  ModelLoader ml;
  BodyInfo_ptr bip = ml.loadBodyInfo( model_url.c_str() );

  bip->loadModelFile( model_url );

}

TEST(ModelLoaderUtilTest, loadModelFile) {

  BodyPtr body(new Body());
  std::string model_url = "./test/model/PA10/pa10.main.wrl";

  EXPECT_TRUE(loadBodyFromModelLoader(body, model_url.c_str(), false));

  body->calcForwardKinematics();
}
