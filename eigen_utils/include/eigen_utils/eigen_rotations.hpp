#pragma once
#include <Eigen/Dense>

namespace eigen_utils {

inline double mod2pi_positive(double vin)
{
    double q = vin / (2*M_PI) + 0.5;
    int qi = (int) q;

    return vin - qi*2*M_PI;
}

/** Map v to [-PI, PI] **/
inline double mod2pi(double vin)
{
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

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

double unwrap(const double& previous_angle, const double& new_angle);

void convertAndUnwrapYaw(const Eigen::Quaterniond& q, const double& yaw_prev, Eigen::Vector3d& rpy);

} // namespace eigen_utils


