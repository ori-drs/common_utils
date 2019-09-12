#ifndef __eigen_rigidbody_h__
#define __eigen_rigidbody_h__

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Define isnan() function for OSX
#if defined(__APPLE__)
#if (__GNUC__ >= 4)
//#define isnan(X) __inline_isnan(X)
#define isnan(X) std::isnan(X)
#else
#define isnan(X) __isnan(X)
#endif
#endif

namespace eigen_utils {

/*
 * returns the skew symmetric matrix corresponding to vec.cross(<other vector>)
 */
Eigen::Matrix3d skewHat(const Eigen::Vector3d & vec);

/**
 * returns the exponential coordinates of quat1 - quat2
 * (quat2.inverse() * quat1)
 */
Eigen::Vector3d subtractQuats(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2);

Eigen::Vector3d subtractRotations(const Eigen::Matrix3d & rot1, const Eigen::Matrix3d & rot2);

void quaternionToBotDouble(double bot_quat[4], const Eigen::Quaterniond & eig_quat);

void botDoubleToQuaternion(Eigen::Quaterniond & eig_quat, const double bot_quat[4]);

Eigen::Quaterniond setQuatEulerAngles(const Eigen::Vector3d & eulers);

Eigen::Vector3d getEulerAngles(const Eigen::Quaterniond & quat);
Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d & rot);
inline Eigen::Vector3d getEulerAnglesDeg(const Eigen::Quaterniond& quat) {
    return getEulerAngles(quat) * 180.0 / M_PI;
}

inline Eigen::Vector3d getEulerAnglesDeg(const Eigen::Matrix3d& rot) {
    return getEulerAngles(rot) * 180.0 / M_PI;
}

Eigen::Affine3d getTransTwistUnscaled(const Eigen::Vector3d & unscaledAngularVelocity,
    const Eigen::Vector3d & unscailedLinearVelocity);
Eigen::Affine3d getTransTwist(const Eigen::Vector3d & angularVelocity, const Eigen::Vector3d & linearVelocity,
    double time);

const double g_val = 9.80665; //gravity
const double rho_val = 1.2; //air density kg/m^3
const Eigen::Vector3d g_vec = -g_val * Eigen::Vector3d::UnitZ(); //ENU gravity vector

/**
 * Basic Rigid Body State representation
 *
 * The chi part of the state vector represents attitude perterbations (exponential coordinates)
 *
 * velocity, angular velocity, and acceleration are tracked in body coordiates
 * acceleration is the "sensed" acceleration including the gravity vector (what an IMU onboard would measure)
 */
class RigidBodyState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum {
    angular_velocity_ind = 0,
    velocity_ind = 3,
    chi_ind = 6,
    position_ind = 9,
    acceleration_ind = 12,
    basic_num_states = 15
  };

  Eigen::VectorXd vec;
  int64_t utime;
  Eigen::Quaterniond quat;

protected:
  RigidBodyState(int state_dim) :
      vec(Eigen::VectorXd::Zero(state_dim)), utime(0), quat(Eigen::Quaterniond::Identity())
  {

  }

public:
  RigidBodyState() :
      vec(Eigen::VectorXd::Zero((int) basic_num_states)), utime(0), quat(Eigen::Quaterniond::Identity())
  {

  }

  RigidBodyState(const Eigen::VectorXd & arg_vec) :
      vec(arg_vec), utime(0), quat(Eigen::Quaterniond::Identity())
  {
    this->chiToQuat();
  }

  RigidBodyState(const Eigen::VectorXd & arg_vec, const Eigen::Quaterniond & arg_quat) :
      vec(arg_vec), utime(0), quat(arg_quat)
  {
  }

  void integrateForwardConstantVelocity(double time)
  {
    Eigen::Affine3d trans = getTransTwist(angularVelocity(), velocity(), time);
    this->orientation() = this->orientation() * trans.rotation();
    this->position() = this->position() + this->orientation() * trans.translation();
  }

  /**
   * phi, theta, psi (roll, pitch, yaw)
   */
  Eigen::Vector3d getEulerAngles() const
  {
    return eigen_utils::getEulerAngles(this->quat);
  }

  /**
   * phi, theta, psi (roll, pitch, yaw)
   */
  void setQuatEulerAngles(const Eigen::Vector3d & eulers)
  {
    this->quat = eigen_utils::setQuatEulerAngles(eulers);
  }



  typedef Eigen::Block<Eigen::VectorXd, 3, 1> Block3Element;
  typedef const Eigen::Block<const Eigen::VectorXd, 3, 1> ConstBlock3Element;

  inline Block3Element velocity()
  {
    return vec.block<3, 1>(RigidBodyState::velocity_ind, 0);
  }

  inline Block3Element chi()
  {
    return vec.block<3, 1>(RigidBodyState::chi_ind, 0);
  }

  inline Block3Element position()
  {
    return vec.block<3, 1>(RigidBodyState::position_ind, 0);
  }

  inline Block3Element angularVelocity()
  {
    return vec.block<3, 1>(RigidBodyState::angular_velocity_ind, 0);
  }

  inline Block3Element acceleration()
  {
    return vec.block<3, 1>(RigidBodyState::acceleration_ind, 0);
  }

  inline Eigen::Quaterniond & orientation()
  {
    return this->quat;
  }

  inline const Eigen::Quaterniond & orientation() const
  {
    return this->quat;
  }

  //const returns
  inline ConstBlock3Element velocity() const
  {
    return vec.block<3, 1>(RigidBodyState::velocity_ind, 0);
  }

  inline ConstBlock3Element chi() const
  {
    return vec.block<3, 1>(RigidBodyState::chi_ind, 0);
  }

  inline ConstBlock3Element position() const
  {
    return vec.block<3, 1>(RigidBodyState::position_ind, 0);
  }

  inline ConstBlock3Element angularVelocity() const
  {
    return vec.block<3, 1>(RigidBodyState::angular_velocity_ind, 0);
  }

  inline ConstBlock3Element acceleration() const
  {
    return vec.block<3, 1>(RigidBodyState::acceleration_ind, 0);
  }

  inline Eigen::Vector3d accelerationGlobal() const
  {
    return orientation() * acceleration();
  }

  inline Eigen::Vector3d accelerationGlobalNoGravity() const
  {
    return orientation() * acceleration() + g_vec;
  }

  inline Eigen::Vector3d velocityGlobal() const
  {
    return orientation() * velocity();
  }

  /**
   * aligns the body x axis with the velocity vector
   */
  void alignVelocityXAxis(double vel_tol = .01)
  {
    Eigen::Quaterniond wind_axes_to_body_axes = Eigen::Quaterniond::Identity(); //transforms vectors in wind frame to vectors in body frame
    if (velocity().norm() > vel_tol) {
      wind_axes_to_body_axes.setFromTwoVectors(Eigen::Vector3d::UnitX(), velocity());
    }

    Eigen::Matrix3d body_axes_to_wind_axes = wind_axes_to_body_axes.toRotationMatrix().transpose();

    angularVelocity() = body_axes_to_wind_axes * angularVelocity();
    velocity() = body_axes_to_wind_axes * velocity();
    acceleration() = body_axes_to_wind_axes * acceleration();
    orientation() = orientation() * wind_axes_to_body_axes;
  }

  void chiToQuat()
  {
    double chi_norm = this->chi().norm();
    if (chi_norm > .000001) { //tolerance check
      Eigen::Quaterniond dquat;
      dquat = Eigen::AngleAxisd(chi_norm, this->chi() / chi_norm);
      this->quat *= dquat;
      this->chi() = Eigen::Vector3d::Zero();
    }
  }

  void quatToChi()
  {
    this->chi() = eigen_utils::subtractQuats(this->quat, Eigen::Quaterniond::Identity());
    this->quat = Eigen::Quaterniond::Identity();
  }

  /**
   * add state on right (postmultiplies orientation)
   */
  void addState(const RigidBodyState & state_to_add)
  {
    this->vec += state_to_add.vec;
    this->chiToQuat();
    this->quat *= state_to_add.quat;
  }

  /**
   * subtract state (premultiplies inverse of state_to_subtract.quat
   */
  void subtractState(const RigidBodyState & state_to_subtract)
  {
    this->vec -= state_to_subtract.vec;
    this->quat = state_to_subtract.quat.inverse() * this->quat;
  }



  bool hasNan() const
  {
    for (int ii = 0; ii < vec.rows(); ii++) {
      if (std::isnan(this->vec(ii)))
        return true;
    }
    for (int ii = 0; ii < 4; ii++) {
      if (std::isnan(this->quat.coeffs()(ii)))
        return true;
    }

    return false;
  }

  static Eigen::Vector3i angularVelocityInds()
  {
    return Eigen::Vector3i::LinSpaced(angular_velocity_ind, angular_velocity_ind + 2);
  }

  static Eigen::Vector3i velocityInds()
  {
    return Eigen::Vector3i::LinSpaced(velocity_ind, velocity_ind + 2);
  }

  static Eigen::Vector3i chiInds()
  {
    return Eigen::Vector3i::LinSpaced(chi_ind, chi_ind + 2);
  }

  static Eigen::Vector3i positionInds()
  {
    return Eigen::Vector3i::LinSpaced(position_ind, position_ind + 2);
  }
};

std::ostream& operator<<(std::ostream& output, const RigidBodyState & state);

}

#endif
