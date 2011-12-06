#ifndef __eigen_lcmgl_h__
#define __eigen_lcmgl_h__

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>
#include <iostream>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_core/bot_core.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace eigen_utils {

void bot_lcmgl_vertex3d(bot_lcmgl_t * lcmgl, Eigen::Vector3d & vec);

void bot_lcmgl_cov_ellipse(bot_lcmgl_t * lcmgl, const Eigen::Matrix2d & cov, const Eigen::Vector3d & mu3d,
    double scale = 1, bool fill = false);

void bot_lcmgl_cov_ellipse(bot_lcmgl_t * lcmgl, const Eigen::Matrix2d & cov, const Eigen::Vector2d & mu2d,
    double scale = 1, bool fill = false);

void bot_lcmgl_cov_ellipse_3d(bot_lcmgl_t * lcmgl, const Eigen::Matrix3d & cov, const Eigen::Vector3d & mu =
    Eigen::Vector3d::Zero(),  double scale = 1);

//use this instead of lcmgl rotate frame
void bot_lcmgl_mult_quat_pos(bot_lcmgl_t * lcmgl, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & pos);

}

#endif