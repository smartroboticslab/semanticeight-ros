//
// Created by anna on 24/05/19.
//

#include <gtest/gtest.h>
#include <eigen-checks/gtest.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "supereight_ros/supereight_ros.hpp"

namespace se{
// class SupereightRosTest: public::testing::Test{
//  protected:
//   SupereightRosTest()
//   : {}
// };

// TODO Test interpolatePose

//GIVEN

// WHEN

// THEN
class interpolateTest : public ::testing::Test {
 public:
  geometry_msgs::TransformStamped &pre_transformation;
  geometry_msgs::TransformStamped &post_transformation;
  int64_t img_time_stamp;
  string actualString;
  string wrongString;
  string expectString;

  char *actualValTrue;
  char *actualValFalse;
  char *expectVal;

  // Use this method to set up any state that you need for all of your tests
  void SetUp() override {
    actualString = "hello gtest";
    wrongString = "hello world";
    expectString = "hello gtest";

    actualValTrue = new char[actualString.size() + 1];
    strncpy(actualValTrue, actualString.c_str(), actualString.size() + 1);

    actualValFalse = new char[wrongString.size() + 1];
    strncpy(actualValFalse, wrongString.c_str(), wrongString.size() + 1);

    expectVal = new char[expectString.size() + 1];
    strncpy(expectVal, expectString.c_str(), expectString.size() + 1);
  }

  // Use this method to clean up any memory, network etc. after each test
  void TearDown() override {
    delete[] actualValTrue;
    delete[] actualValFalse;
    delete[] expectVal;
  }
};

TEST(calculateAlphaTest, calculateAlphaTest_timeEqual_Test  ){
  //GIVEN

  // WHEN

  // THEN
}

}