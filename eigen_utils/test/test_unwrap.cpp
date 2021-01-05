#include <gtest/gtest.h>
#include "eigen_utils/eigen_rotations.hpp"

using namespace eigen_utils;

TEST(Unwrap, DISABLED_cornerCases){

  // TODO fill in the angles
  std::vector<double> prev_angles; // = {0, M_PI, 10 };
  std::vector<double> angles;
  std::vector<double> expected_outputs;

  EXPECT_TRUE(prev_angles.size() == angles.size());
  EXPECT_TRUE(angles.size() == expected_outputs.size());

  for(size_t i = 0; i < angles.size(); ++i){
    double res = unwrap(prev_angles[i], angles[i]);
    EXPECT_DOUBLE_EQ(res, expected_outputs[i]);
  }

}
