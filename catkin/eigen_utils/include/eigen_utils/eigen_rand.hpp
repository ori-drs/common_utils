#ifndef __eigen_rand_h__
#define __eigen_rand_h__

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>
#include <random>

namespace eigen_utils {

//todo see about replacing templated functions with MatrixBase arguments and then checking for num rows and cols
/**
 * Fill an Eigen vector with gaussian random numbers with mu=0, Sigma = I
 */
template<typename scalarType, int N>
inline void randn_identity(Eigen::Matrix<scalarType, N, 1> & vec)
{
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{0,1};

  for (int ii = 0; ii < vec.size(); ii++) {
    vec(ii) = d(gen);
  }
}

template<typename scalarType, int N>
inline Eigen::Matrix<scalarType, N, 1> randn_chol(const Eigen::Matrix<scalarType, N, N> & chol_decomp_cov)
{
  Eigen::Matrix<scalarType, N, 1> vec;
  randn_identity(vec);
  return chol_decomp_cov * vec;
}

template<typename scalarType, int N>
inline Eigen::Matrix<scalarType, N, 1> randn(const Eigen::Matrix<scalarType, N, N> & cov)
{
  Eigen::Matrix<scalarType, N, N> chol_decomp = cov.llt().matrixL();
  return randn_chol(chol_decomp);
}

/**
 * unnormalized log likelihood
 */
template<typename scalarType, int N>
inline scalarType loglike_unnormalized(const Eigen::Matrix<scalarType, N, 1> & x
    , const Eigen::Matrix<scalarType, N, 1> & mu
    , const Eigen::Matrix<scalarType, N, N> & sigma)
{
  Eigen::Matrix<scalarType, N, 1> diff = mu - x;
  return -0.5 * diff.transpose() * sigma.ldlt().solve(diff);
}

/**
 * unnormalized log likelhihood using information matrix
 */
template<typename scalarType, int N>
inline scalarType loglike_information_unnormalized(const Eigen::Matrix<scalarType, N, 1> & x
    ,const Eigen::Matrix<scalarType, N, 1> & mu , const Eigen::Matrix<scalarType, N, N> & sigma_inv)
{
  Eigen::Matrix<scalarType, N, 1> diff = mu - x;
  return -0.5 * diff.transpose() * sigma_inv * diff;
}

template<typename DerivedX, typename DerivedMU, typename DerivedSigma>
double loglike_normalized(const Eigen::MatrixBase<DerivedX> & x, const Eigen::MatrixBase<DerivedMU> & mu,
    const Eigen::MatrixBase<DerivedSigma> & sigma)
{
  Eigen::VectorXd diff = mu - x;
  return -log(sigma.determinant()) - diff.transpose() * sigma.ldlt().solve(diff);
}

template<typename scalarType, int N>
inline scalarType normpdf(const Eigen::Matrix<scalarType, N, 1> & x, const Eigen::Matrix<scalarType, N, 1> & mu
    , const Eigen::Matrix<scalarType, N, N> & sigma)
{
  Eigen::Matrix<scalarType, N, 1> diff = mu - x;

  scalarType exponent = -0.5 * diff.transpose() * sigma.ldlt().solve(diff);

  return exp(exponent) / (pow(2 * M_PI, ((scalarType) N) / 2.0) * pow(sigma.determinant(), 0.5));
}

void fitParticles(const Eigen::MatrixXd & state_samples
    , const Eigen::VectorXd & weights, Eigen::VectorXd & mean
    , Eigen::MatrixXd & covariance);

// taken from libbot
// TODO use from libbot once is cleaned up from LCM

/* random number between [0, 1) */
inline float randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}

/* random number between (-1, 1) */
inline float signed_randf()
{
    return randf()*2.0 - 1.0;
}
/* random number between [mi, ma ] */
inline float randf_in_range(float mi, float ma)
{
    return randf()*(ma-mi) + mi;
}

}
#endif
