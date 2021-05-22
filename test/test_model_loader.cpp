#include <gtest/gtest.h>

#include "ModelLoader.hpp"
#include "ModelLoaderUtil.hpp"

using namespace hrp;

#define DEG2RAD(deg)

inline double deg2rad(const double& degree ) {
  return degree * M_PI / 180.0;
}

inline double rad2deg(const double& radian ) {
  return radian * 180.0 / M_PI;
}

TEST(BodyInfoTest, loadModeFile) {
  std::string model_url = "./test/model/sample.wrl";
  ModelLoader ml;
  BodyInfo_ptr bip = ml.loadBodyInfo( model_url.c_str() );

  EXPECT_NO_THROW( bip->loadModelFile( model_url ) );

}

TEST(ModelLoaderUtilTest, loadModelFile) {

  BodyPtr body(new Body());
  std::string model_url = "./test/model/sample.wrl";

  EXPECT_TRUE(loadBodyFromModelLoader(body, model_url.c_str(), false));

}

TEST(ModelLoaderUtilTest, calcForwardKinematics) {

  BodyPtr body(new Body());
  std::string model_url = "./test/model/PA10/pa10.main.wrl";

  EXPECT_TRUE(loadBodyFromModelLoader(body, model_url.c_str(), false));

  EXPECT_NO_THROW( body->calcForwardKinematics() );

  const std::size_t JOINT_NUM = 7;
  const std::size_t POSE_PATTERN_NUM = 11;
  // test-input of joint angle degree
  const double test_joint_angle_deg [POSE_PATTERN_NUM][JOINT_NUM] =
    {
      {  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,   0.0 },
      { 10.0,   0.0,   0.0,  0.0,   0.0,   0.0,   0.0 },
      {  0.0,  20.0,   0.0,  0.0,   0.0,   0.0,   0.0 },
      {  0.0,   0.0,  30.0,  0.0,   0.0,   0.0,   0.0 },
      {  0.0,   0.0,   0.0, 40.0,   0.0,   0.0,   0.0 },
      {  0.0,   0.0,   0.0,  0.0,  50.0,   0.0,   0.0 },
      {  0.0,   0.0,   0.0,  0.0,   0.0,  60.0,   0.0 },
      {  0.0,   0.0,   0.0,  0.0,   0.0,   0.0,  70.0 },
      { 10.0,   5.0,  30.0,  5.0,  60.0,  30.0,  10.0 },
      { -1.0,  -5.0,  -3.0, -5.0, -10.0,  -2.0,  -4.0 },
      { 31.0,  25.0,   1.5, 10.0,  30.5,  12.2,   7.7 }
    };

  // End Effector Name
  const std::string EE_NAME = "J7";

  std::cout << std::endl;

  // test Forward Kinematics
  for(std::size_t i=0; i<POSE_PATTERN_NUM; i++) {

    std::cout << "[ ";
    for(std::size_t jid=0; jid<JOINT_NUM; jid++) {
      // input joint angle
      body->joint(jid)->q = deg2rad( test_joint_angle_deg[i][jid] );
      std::cout << test_joint_angle_deg[i][jid] << " ";
    }
    std::cout << "] (joint agnle[deg])" << std::endl;

    // calculate Forward Kinematics
    EXPECT_NO_THROW( body->calcForwardKinematics() );

    std::cout << " -> ";

    // output position from WORLD
    std::cout << "[ "
              << body->link(EE_NAME)->p[0] << " "  // x
              << body->link(EE_NAME)->p[1] << " "  // y
              << body->link(EE_NAME)->p[2] << " "; // z

    // output orientation from WORLD (Rotation Matrix -> Angle Axis)
    // Eigen::AngleAxisd aa( body->link(EE_NAME)->R );
    // std::cout << aa.axis()[0] << " "    // axis-x
    //           << aa.axis()[1] << " "    // axis-y
    //           << aa.axis()[2] << " "    // axis-z
    //           << rad2deg( aa.angle() )  // angle
    //           << " ] (axis-xyz anlge[deg])" << std::endl;

    // output orientation from WORLD (Rotation Matrix -> Roll Pitch Yaw)
    std::cout << rad2deg( body->link(EE_NAME)->R.eulerAngles(0,1,2)[0] ) << " " // roll
              << rad2deg( body->link(EE_NAME)->R.eulerAngles(0,1,2)[1] ) << " " // pitch
              << rad2deg( body->link(EE_NAME)->R.eulerAngles(0,1,2)[2] ) << " " // yaw
              << "] (xyz[m] rpy[deg])" << std::endl;

    std::cout << std::endl;
  }

}
