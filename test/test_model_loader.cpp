#include <gtest/gtest.h>
#include "ModelLoaderUtil.hpp"

TEST(ModelLoaderTest, Load) {

  BodyPtr body(new Body());
  std::string model_url = "test/model/PA10/pa10.main.wrl";
  EXPECT_TRUE(loadBodyFromModelLoader(body, model_url.c_str(), false));
}
