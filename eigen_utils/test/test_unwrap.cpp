#include <gtest/gtest.h>
#include "eigen_utils/eigen_rotations.hpp"

using namespace eigen_utils;

TEST(Unwrap, cornerCases){

  std::vector<double> prev_angles = {
    0.,
    0.3300,
    -1.6998,
    -4.7838,
    2.5411,
  };

  std::vector<double> new_angles = {
    1.5622,
    1.0000,
    1.7252,
    1.5580,
    -6.1268,
  };

  std::vector<double> expected_unwrapped_angles = {
    1.5622, // jump of  1.5622 ( 1.5622)
    1.0000, // jump of  1.0000 ( 0.6700)
    -4.5579853071795862, // jump of  3.4250 (-2.8582)
    -4.7251853071795864, // jump of  6.3420 ( 0.0586)
    0.15638530717958687, // jump of -8.6679 (-2.3847)
  };

  EXPECT_TRUE(prev_angles.size() == prev_angles.size());
  EXPECT_TRUE(expected_unwrapped_angles.size() == new_angles.size());

  for(size_t i = 0; i < prev_angles.size(); ++i){
    double unwraped_angle = unwrap(prev_angles[i], new_angles[i]);
    EXPECT_DOUBLE_EQ(unwraped_angle, expected_unwrapped_angles[i]);

//    std::cout.precision(4);
//    std::cout << std::fixed << std::setfill(' ')
//              << std::left  << std::setw(11) << "prev_angles[" << i << "] = "
//              << std::right << std::setw(7)  << prev_angles[i]
//              << std::left  << std::setw(12) << ", new_angles[" << i << "] = "
//              << std::right << std::setw(7)  <<  new_angles[i]
//              << std::left  << std::setw(19) << ": unwraped_angle = "
//              << std::right << std::setw(10) << unwraped_angle
//              << std::endl;
  }
}
