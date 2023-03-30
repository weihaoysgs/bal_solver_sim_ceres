#ifndef BOUNDLEADJUSTMENT_POINT_CAMERA_H
#define BOUNDLEADJUSTMENT_POINT_CAMERA_H
#include <math.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
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
#include "jet.h"
#include "stdlib.h"
using namespace std;
double debug = true;
namespace BoundleAdjustment {
template <int N, int N1, int N2>
class CostFunction
{
 public:
  CostFunction();
  ~CostFunction();
  virtual void computeJacobiandResidual(
      const Eigen::Matrix<double, N1, 1> *params_1,
      const Eigen::Matrix<double, N2, 1> *params_2,
      Eigen::Matrix<double, N, N1> *jacobi_parameter_1,
      Eigen::Matrix<double, N, N2> *jacobi_parameter_2,
      Eigen::Matrix<double, N, 1> *jacobi_residual) = 0;
  virtual void computeResidual(const Eigen::Matrix<double, N1, 1> *params_1,
                               const Eigen::Matrix<double, N2, 1> *params_2,
                               double *ans) = 0;
};
template <int N, int N1, int N2>
CostFunction<N, N1, N2>::CostFunction()
{
}
template <int N, int N1, int N2>
CostFunction<N, N1, N2>::~CostFunction()
{
}

/* The residual_node is the key class to compute the residual and jacobi.
 * It also the most time-cost process in function computeJacobianandResidual()
 * template T used to pass the class T which writen by user and the T must have
 * a function named "Evaluate",and the function Evaluate must be a template
 * function which could compute with both double and jet
 */
template <class T, int N, int N1, int N2>
class Residual_node : public CostFunction<N, N1, N2>
{
 public:
  Residual_node();
  Residual_node(T *costfunction);
  ~Residual_node();
  // compute the jacobian and residual both,it happen in the outloop compute
  T *costfunction_;
  void computeJacobiandResidual(
      const Eigen::Matrix<double, N1, 1> *params_1,
      const Eigen::Matrix<double, N2, 1> *params_2,
      Eigen::Matrix<double, N, N1> *jacobi_parameter_1,
      Eigen::Matrix<double, N, N2> *jacobi_parameter_2,
      Eigen::Matrix<double, N, 1> *jacobi_residual);
  // compute only residual both,it happen in the predict stage.
  void computeResidual(const Eigen::Matrix<double, N1, 1> *params_1,
                       const Eigen::Matrix<double, N2, 1> *params_2,
                       double *ans);
};

template <class T, int N, int N1, int N2>
Residual_node<T, N, N1, N2>::Residual_node()
{
}
template <class T, int N, int N1, int N2>
Residual_node<T, N, N1, N2>::Residual_node(T *costfunction)
    : costfunction_(costfunction)
{
}

template <class T, int N, int N1, int N2>
void Residual_node<T, N, N1, N2>::computeJacobiandResidual(
    const Eigen::Matrix<double, N1, 1> *params_1,
    const Eigen::Matrix<double, N2, 1> *params_2,
    Eigen::Matrix<double, N, N1> *jacobi_parameter_1,
    Eigen::Matrix<double, N, N2> *jacobi_parameter_2,
    Eigen::Matrix<double, N, 1> *jacobi_residual)
{
  // clock_t t1=clock();
  jet<N1 + N2> cameraJet[N1];
  jet<N1 + N2> pointJet[N2];
  for (int i = 0; i < N1; i++)
  {
    cameraJet[i].init((*params_1)[i], i);
  }
  for (int i = 0; i < N2; i++)
  {
    pointJet[i].init((*params_2)[i], i + N1);
  }
  jet<N1 + N2> *residual = new jet<N1 + N2>[N];
  costfunction_->Evaluate(cameraJet, pointJet, residual);
  for (int i = 0; i < N; i++)
  {
    (*jacobi_residual)(i, 0) = residual[i].a;
  }
  for (int i = 0; i < N; i++)
  {
    (*jacobi_parameter_1).row(i) = residual[i].v.head(N1);
    (*jacobi_parameter_2).row(i) = residual[i].v.tail(N2);
  }
  // std::cout << "popint jacobian: \n" << *jacobi_parameter_2;
  delete (residual);
  return;
  /******************end compute jacobi*************************/
}
template <class T, int N, int N1, int N2>
void Residual_node<T, N, N1, N2>::computeResidual(
    const Eigen::Matrix<double, N1, 1> *params_1,
    const Eigen::Matrix<double, N2, 1> *params_2, double *ans)
{
  double parameterD_1[N1];
  Eigen::Map<Eigen::Matrix<double, N1, 1>>(parameterD_1, params_1->rows(), 1) =
      (*params_1);
  double parameterD_2[N2];
  Eigen::Map<Eigen::Matrix<double, N2, 1>>(parameterD_2, params_2->rows(), 1) =
      (*params_2);
  double residual[N];
  costfunction_->Evaluate(parameterD_1, parameterD_2, residual);
  for (int i = 0; i < N; ++i)
  {
    (*ans) += residual[i] * residual[i];
  }
  // std::cout << "Residual: " << *ans << std::endl;
}
}  // namespace BoundleAdjustment

#endif
