#pragma once
/**
 * alpha is (time step)/(filter time constant)
 */
template<typename Type>
class ExponentialFilter {
public:

  ExponentialFilter(const Type & init_val, double alpha) :
      val(init_val), alpha()
  {
  }

  const Type & step(const Type & new_val)
  {
    val = alpha * new_val + (1 - alpha) * val;
  }

  const Type & operator()() const
  {
    return getVal();
  }

  const Type & getVal() const
  {
    return val;
  }

  const double & getAlpha() const
  {
    return alpha;
  }

  void setAlpha(double _alpha)
  {
    alpha = _alpha;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  double alpha;
  Type val;
}
;
