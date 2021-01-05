#include "eigen_utils/eigen_rigidbody.hpp"


namespace eigen_utils {

using namespace Eigen;

std::ostream& operator<<(std::ostream& output, const RigidBodyState & state)
{
  output << "angularVelocity: " << state.angularVelocity().transpose();
  output << ", velocity: " << state.velocity().transpose();
  output << ", chi: " << state.chi().transpose();
  output << ", position: " << state.position().transpose();
  output << ", acceleration: " << state.acceleration().transpose();
  output << ", RPY: " << (state.getEulerAngles().transpose()*180.0/M_PI);
  return output;
}




}

