#ifndef BoundleAdjustment_H_
#define BoundleAdjustment_H_
#include <math.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "Eigen/Eigen"
#include "Eigen/SparseQR"
#include "algorithm"
#include "stdlib.h"
// using namespace std;
/* Like ceres the jet use for autodiff*/
template <int N>
struct jet
{
  Eigen::Matrix<double, N, 1> v;
  double a;
  jet() : a(0.0) {}
  jet(const double& value) : a(value) { v.setZero(); }
  EIGEN_STRONG_INLINE jet(const double& value,
                          const Eigen::Matrix<double, N, 1>& v_)
      : a(value), v(v_)
  {
  }
  jet(const double value, const int index)
  {
    v.setZero();
    a = value;
    v(index, 0) = 1.0;
  }
  void init(const double value, const int index)
  {
    v.setZero();
    a = value;
    v(index, 0) = 1.0;
  }
};
/****************jet overload******************/
// for the camera BA,the autodiff only need overload the operator :jet+jet
// number+jet -jet jet-number jet*jet number/jet jet/jet sqrt(jet) cos(jet)
// sin(jet)  +=(jet) overload jet + jet
template <int N>
inline jet<N> operator+(const jet<N>& A, const jet<N>& B)
{
  return jet<N>(A.a + B.a, A.v + B.v);
}  // end jet+jet

// overload number + jet
template <int N>
inline jet<N> operator+(double A, const jet<N>& B)
{
  return jet<N>(A + B.a, B.v);
}  // end number+jet

// overload jet-number
template <int N>
inline jet<N> operator-(const jet<N>& A, double B)
{
  return jet<N>(A.a - B, A.v);
}
// overload number * jet because jet *jet need A.a *B.v+B.a*A.v.So the number
// *jet is required
template <int N>
inline jet<N> operator*(double A, const jet<N>& B)
{
  return jet<N>(A * B.a, A * B.v);
}
template <int N>
inline jet<N> operator*(const jet<N>& A, double B)
{
  return jet<N>(B * A.a, B * A.v);
}
// overload -jet
template <int N>
inline jet<N> operator-(const jet<N>& A)
{
  return jet<N>(-A.a, -A.v);
}
template <int N>
inline jet<N> operator-(double A, const jet<N>& B)
{
  return jet<N>(A - B.a, -B.v);
}
template <int N>
inline jet<N> operator-(const jet<N>& A, const jet<N>& B)
{
  return jet<N>(A.a - B.a, A.v - B.v);
}
// overload jet*jet
template <int N>
inline jet<N> operator*(const jet<N>& A, const jet<N>& B)
{
  return jet<N>(A.a * B.a, B.a * A.v + A.a * B.v);
}
// overload number/jet
template <int N>
inline jet<N> operator/(double A, const jet<N>& B)
{
  return jet<N>(A / B.a, -A * B.v / (B.a * B.a));
}
// overload jet/jet
template <int N>
inline jet<N> operator/(const jet<N>& A, const jet<N>& B)
{
  // This uses:
  //
  //   a + u   (a + u)(b - v)   (a + u)(b - v)
  //   ----- = -------------- = --------------
  //   b + v   (b + v)(b - v)        b^2
  //
  // which holds because v*v = 0.
  const double a_inverse = 1.0 / B.a;
  const double abyb = A.a * a_inverse;
  return jet<N>(abyb, (A.v - abyb * B.v) * a_inverse);
}
// sqrt(jet)
template <int N>
inline jet<N> sqrt(const jet<N>& A)
{
  double t = std::sqrt(A.a);

  return jet<N>(t, 1.0 / (2.0 * t) * A.v);
}
// cos(jet)
template <int N>
inline jet<N> cos(const jet<N>& A)
{
  return jet<N>(std::cos(A.a), -std::sin(A.a) * A.v);
}
template <int N>
inline jet<N> sin(const jet<N>& A)
{
  return jet<N>(std::sin(A.a), std::cos(A.a) * A.v);
}
template <int N>
inline bool operator>(const jet<N>& f, const jet<N>& g)
{
  return f.a > g.a;
}

#endif  // BoundleAdjustment_JET_H_
