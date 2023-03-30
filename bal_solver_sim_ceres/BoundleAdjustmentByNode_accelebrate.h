#include <math.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <algorithm>
#include <iomanip>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "Eigen/SparseQR"
#include "algorithm"
#include "jet.h"
#include "point_camera.h"
#include "stdlib.h"
#define MAXSTEP 1e8
#define MINSTEP 1e-9
#define MINDIF 1e-6
#define PARAMETERMIN 1e-8
#define max_consecutive_nonmonotonic_steps 0
#define OUTTER_ITERATION_NUMBER 50
#define INNER_ITERATION_NUMBER 50
#define INITMIU 1e4
using namespace std;
/* The class Problem used to solve a boundleAdjustment problem.For faster the
 * compute progress,use a template to init the matrix in Problem. Because the
 * static matrix can compute faster than dynamic matrix.Notice the schur matrix
 * S could not known before the problem begin to solve.So the Schur_A and
 * Schur_B must be dynamic. The solve process can be this: First use function
 * addParameterBlock to add the parameter_camera and parameter_point to this
 * problem. Then in function solve ,we compute the residual and jacobi first and
 * use function  pre_process() to set the schur matrix's size and build a right
 * pair of camera.then init_scaling can init the scaling params once. These
 * three functions only compute once.Then do the LM loop method: First scaling
 * the jacobi matrix,then compute the hessian and write camera's hessian to
 * Schur Matrix.Also the camera's Residual(=Ji^T*Ui). Then do the
 * schur_complement to compute the delta Camera After that we get the delta step
 * of camera,then use (deltaP) = hessianP^-1*(Vi - W^T*deltaC) -->delta step of
 * Point. Compute the model_cost and then the real cost.If the
 * relative_decrease>0.001,we get a success step,update this and continue the
 * process until limit reached.
 *
 */
namespace BoundleAdjustment {
template <int N, int N1, int N2>
class Problem
{
 public:
  Problem();
  ~Problem();
  struct Residual_block
  {
    Residual_block(int a, int b,
                   BoundleAdjustment::CostFunction<N, N1, N2> *node)
        : parameter_a(a), parameter_b(b)
    {
      jacobi_parameter_1.setZero();
      jacobi_parameter_2.setZero();
      hessian_W.setZero();
      residual.setZero();
      residual_node = node;
    }
    int parameter_a;
    int parameter_b;
    /*The residual_node use to make the costfunction feasible,CostFunction is a
     * virtual class and its sub class Residual_node is a template class which
     * has a class T.Then class T can be writen in main and offer the key
     * function(Evaluate) to compute the residual and jacobian. Virtual class is
     * used for this trick even it consume some compute time.
     */
    BoundleAdjustment::CostFunction<N, N1, N2> *residual_node;
    Eigen::Matrix<double, N, N1> jacobi_parameter_1;
    Eigen::Matrix<double, N, N2> jacobi_parameter_2;
    Eigen::Matrix<double, N1, N2> hessian_W;
    Eigen::Matrix<double, N, 1> residual;
  };
  template <int n>
  struct Parameter
  {
    Parameter()
    {
      jacobi_scaling.setZero();
      hessian.setZero();
      hessian_inverse.setZero();
      params.setZero();
      candidate.setZero();
      delta.setZero();
      residual.setZero();
    }
    Eigen::Matrix<double, n, 1> params;
    Eigen::Matrix<double, n, 1> candidate;
    Eigen::Matrix<double, n, 1> delta;
    Eigen::Matrix<double, n, 1> residual;
    Eigen::Matrix<double, n, 1> jacobi_scaling;
    Eigen::Matrix<double, n, n> hessian;
    // The hessian_inverse used just for time saving.
    Eigen::Matrix<double, n, n> hessian_inverse;
  };
  vector<Residual_block *> residual_block;
  vector<Parameter<N1> *> parameter_1_vector;
  vector<Parameter<N2> *> parameter_2_vector;
  map<double *, int> parameter_1_map;
  map<double *, int> parameter_2_map;
  int parameter_a_size;
  Eigen::MatrixXd Schur_A;
  Eigen::VectorXd Schur_B;
  bool update_parameter(double *step);
  bool checkParameter_1(double *parameter_1);
  bool checkParameter_2(double *parameter_2);
  void addParameterBlock(
      double *parameter_1, double *parameter_2,
      BoundleAdjustment::CostFunction<N, N1, N2> *costfunction);
  void pre_process();
  void init_scaling();
  void solve();
  inline void schur_complement();
  // The function post_process used to copy the optimal pamameter to the double
  // array the user given.
  void post_process();
  static bool cmp(Residual_block *A, Residual_block *B);
  vector<vector<Residual_block *>> parameter_2_link;
  void removeParam(int param_id);  // this function will add in next version
  void out_schur_elimilate();      // this function will add in next version
  void incremental_optimal();      // this function will add in next version
};

template <int N, int N1, int N2>
Problem<N, N1, N2>::Problem() : parameter_a_size(0)
{
}
template <int N, int N1, int N2>
Problem<N, N1, N2>::~Problem()
{
  // delete the point we create
  for (int i = 0; i < parameter_1_vector.size(); ++i)
  {
    delete (parameter_1_vector[i]);
  }
  for (int i = 0; i < parameter_2_vector.size(); ++i)
  {
    delete (parameter_2_vector[i]);
  }
  for (int i = 0; i < residual_block.size(); ++i)
  {
    delete (residual_block[i]);
  }
}
template <int N, int N1, int N2>
bool Problem<N, N1, N2>::cmp(Residual_block *A, Residual_block *B)
{
  // Order the residual_block point by the camera id
  if (A->parameter_a < B->parameter_a) return true;
  return false;
}
template <int N, int N1, int N2>
void Problem<N, N1, N2>::pre_process()
{
  /* pre_process need to be done before schur_complement
   * in the process,the numbers of parameter_a and parameter_b are known.Since
   * schur_complement cost much time we need to pre_process the struct of point
   * to camera. Notice that if the camera and point are not changed,the map
   * would only construct once. use the parameter_2_link,which is a link_list as
   * Point--->residual_node--->next residual_node
   *
   *                                                    |
   *                                                    |
   *                                                    V
   *                                                nextPoint
   * In each point we can find the pair<int Ci,int Cj>. Since the schur matrix
   * is Symmetric Matrix,we only need Ci<Cj so in this function we order the
   * link_list by camera id. Compute with the link happened in the function
   * schur_complement().
   */
  Schur_A.resize(parameter_a_size, parameter_a_size);
  Schur_B.resize(parameter_a_size);
  Schur_A.setZero();
  Schur_B.setZero();
  for (int i = 0; i < parameter_2_link.size(); ++i)
  {
    sort(parameter_2_link[i].begin(), parameter_2_link[i].end(), cmp);
  }
}
template <int N, int N1, int N2>
bool Problem<N, N1, N2>::update_parameter(double *step)
{
  // from cadidte to params and compute the step norm
  double step_norm = 0.0;
  for (int i = 0; i < parameter_1_vector.size(); ++i)
  {
    parameter_1_vector[i]->params = parameter_1_vector[i]->candidate;
    parameter_1_vector[i]->hessian.setZero();
    parameter_1_vector[i]->residual.setZero();
    step_norm = step_norm + parameter_1_vector[i]->delta.norm();
  }
  for (int i = 0; i < parameter_2_vector.size(); ++i)
  {
    parameter_2_vector[i]->params = parameter_2_vector[i]->candidate;
    parameter_2_vector[i]->hessian.setZero();
    parameter_2_vector[i]->residual.setZero();
    step_norm = step_norm + parameter_2_vector[i]->delta.norm();
  }
  (*step) = step_norm;
  if (step_norm < PARAMETERMIN)
  {
    return false;
  }
  return true;
}
template <int N, int N1, int N2>
bool Problem<N, N1, N2>::checkParameter_1(double *parameter_1)
{
  // we check if the double array had in the map<double,camera>
  if (parameter_1_map.find(parameter_1) != parameter_1_map.end()) return true;
  return false;
}
template <int N, int N1, int N2>
bool Problem<N, N1, N2>::checkParameter_2(double *parameter_2)
{
  // we check if the double array had in the map<double,point>
  if (parameter_2_map.find(parameter_2) != parameter_2_map.end()) return true;
  return false;
}
template <int N, int N1, int N2>
void Problem<N, N1, N2>::addParameterBlock(
    double *parameter_1, double *parameter_2,
    BoundleAdjustment::CostFunction<N, N1, N2> *new_residual_node)
{
  // for saving time,we don't check the camera and point if there is
  // already a same tuple.So the input tuple must be different.

  if (!checkParameter_1(parameter_1))
  {
    Parameter<N1> *new_parameter = new Parameter<N1>();
    new_parameter->params =
        Eigen::Map<Eigen::Matrix<double, N1, 1>>(parameter_1, N1);
    parameter_1_map.insert(
        std::pair<double *, int>(parameter_1, parameter_1_vector.size()));
    parameter_1_vector.push_back(new_parameter);
    parameter_a_size = parameter_a_size + N1;
  }
  if (!checkParameter_2(parameter_2))
  {
    Parameter<N2> *new_parameter = new Parameter<N2>();
    new_parameter->params =
        Eigen::Map<Eigen::Matrix<double, N2, 1>>(parameter_2, N2);
    parameter_2_map.insert(
        std::pair<double *, int>(parameter_2, parameter_2_vector.size()));
    parameter_2_vector.push_back(new_parameter);
    vector<Residual_block *> parameter_2_list;
    parameter_2_link.push_back(parameter_2_list);
  }
  Residual_block *block =
      new Residual_block(parameter_1_map[parameter_1],
                         parameter_2_map[parameter_2], new_residual_node);
  int id = block->parameter_b;
  parameter_2_link[id].push_back(block);
  residual_block.push_back(block);
}

/****************************************schur
 * complement********************************/
template <int N, int N1, int N2>
void Problem<N, N1, N2>::schur_complement()
{
  /* schur_complement and the jacobi compute process is two time-cost processes.
   * We use the point_link to find the pair of camera which share one same
   * point. Notice Sij = sum(Wi^T*P^-1*Wj),so if we search pair by point ,the
   * Wi^T*P^-1 can keep in the second loop. As the schur matrix is symmetric.We
   * only compute the upper of S. By the way, the Schur_B can be computed by the
   * same process. finally because the Schur_A is a positive symmetric matrix,we
   * can use the llt() to solve delta camera.
   */
  int length_camera = parameter_1_vector.size();
  int parameter_2_link_size = parameter_2_link.size();

  for (int i = 0; i < parameter_2_link_size; ++i)
  {
    int inner_size = parameter_2_link[i].size();
    for (int j = 0; j < inner_size; ++j)
    {
      int id_1 = parameter_2_link[i][j]->parameter_a;
      Eigen::Matrix<double, N1, N2> WT_Einv;
      WT_Einv.noalias() = parameter_2_link[i][j]->hessian_W.lazyProduct(
          parameter_2_vector[i]->hessian_inverse);
      Schur_B.segment<N1>(id_1 * N1).noalias() -=
          WT_Einv.lazyProduct(parameter_2_vector[i]->residual);
      for (int k = j; k < inner_size; ++k)
      {
        int id_2 = parameter_2_link[i][k]->parameter_a;
        Schur_A.block<N1, N1>(id_1 * N1, id_2 * N1).noalias() -=
            WT_Einv.lazyProduct(parameter_2_link[i][k]->hessian_W.transpose());
      }
    }
  }
  Schur_B = Schur_A.selfadjointView<Eigen::Upper>().llt().solve(Schur_B);
  for (int i = 0; i < length_camera; ++i)
  {
    parameter_1_vector[i]->delta = Schur_B.segment<N1>(i * N1);
  }
}

/****************************************end schur
 * complement********************************/
template <int N, int N1, int N2>
void Problem<N, N1, N2>::init_scaling()
{
  // jacobi_scaling = 1/(1+sqrt(jacobi_scaling )) plus the one to avoid / zero
  int parameter_1_length = parameter_1_vector.size();
  int parameter_2_length = parameter_2_vector.size();
  for (int i = 0; i < parameter_1_length; ++i)
  {
    parameter_1_vector[i]->jacobi_scaling =
        1.0 / (1.0 + sqrt(parameter_1_vector[i]->jacobi_scaling.array()));
  }
  for (int i = 0; i < parameter_2_length; ++i)
  {
    parameter_2_vector[i]->jacobi_scaling =
        1.0 / (1.0 + sqrt(parameter_2_vector[i]->jacobi_scaling.array()));
  }
}
template <int N, int N1, int N2>
void Problem<N, N1, N2>::solve()
{
  int outcount = 0;
  double minimum_cost = -1;
  double model_cost = 0.0;
  int num_consecutive_nonmonotonic_steps = 0;
  double accumulated_reference_model_cost_change = 0;
  double accumulated_candidate_model_cost_change = 0;
  double current_cost;
  double candidate_cost;
  double reference_cost;
  double Miu = INITMIU;
  double v0 = 2.0;
  clock_t t1 = clock();
  int parameter_1_vector_length = parameter_1_vector.size();
  int parameter_2_vector_length = parameter_2_vector.size();
  int residual_node_length = residual_block.size();
  double residual_cost = 0.0;
  for (int i = 0; i < residual_node_length; ++i)
  {
    residual_block[i]->residual_node->computeJacobiandResidual(
        &parameter_1_vector[residual_block[i]->parameter_a]->params,
        &parameter_2_vector[residual_block[i]->parameter_b]->params,
        &residual_block[i]->jacobi_parameter_1,
        &residual_block[i]->jacobi_parameter_2, &residual_block[i]->residual);
    residual_cost = residual_cost + residual_block[i]->residual.squaredNorm();
    parameter_1_vector[residual_block[i]->parameter_a]->jacobi_scaling +=
        residual_block[i]->jacobi_parameter_1.colwise().squaredNorm();
    parameter_2_vector[residual_block[i]->parameter_b]->jacobi_scaling +=
        residual_block[i]->jacobi_parameter_2.colwise().squaredNorm();
  }
  residual_cost /= 2;
  pre_process();
  init_scaling();
  cout << "iteration|    new_residual    |    old_residual    |    step_norm   "
          " |    radius    |    iter time    "
       << endl;
  while (outcount < OUTTER_ITERATION_NUMBER)
  {
    for (typename std::vector<
             BoundleAdjustment::Problem<N, N1, N2>::Residual_block *>::iterator
             it = residual_block.begin();
         it != residual_block.end(); ++it)
    {
      (*it)->jacobi_parameter_1 = (*it)->jacobi_parameter_1.array().rowwise() *
                                  parameter_1_vector[(*it)->parameter_a]
                                      ->jacobi_scaling.transpose()
                                      .array();
      (*it)->jacobi_parameter_2 = (*it)->jacobi_parameter_2.array().rowwise() *
                                  parameter_2_vector[(*it)->parameter_b]
                                      ->jacobi_scaling.transpose()
                                      .array();
      (*it)->hessian_W.noalias() =
          (*it)->jacobi_parameter_1.transpose().lazyProduct(
              (*it)->jacobi_parameter_2);
    }
    ++outcount;
    double totaltime = (double)(clock() - t1) / CLOCKS_PER_SEC;
    t1 = clock();
    if (outcount <= 1)
    {
      candidate_cost = residual_cost;
      reference_cost = residual_cost;
    }
    current_cost = residual_cost;
    accumulated_candidate_model_cost_change += model_cost;
    accumulated_reference_model_cost_change += model_cost;
    if (outcount == 1 || current_cost < minimum_cost)
    {
      minimum_cost = current_cost;
      num_consecutive_nonmonotonic_steps = 0;
      candidate_cost = current_cost;
      accumulated_candidate_model_cost_change = 0;
    }
    else
    {
      ++num_consecutive_nonmonotonic_steps;
      if (current_cost > candidate_cost)
      {
        candidate_cost = current_cost;
        accumulated_candidate_model_cost_change = 0.0;
      }
    }
    if (num_consecutive_nonmonotonic_steps ==
        max_consecutive_nonmonotonic_steps)
    {
      reference_cost = candidate_cost;
      accumulated_reference_model_cost_change =
          accumulated_candidate_model_cost_change;
    }
    /*****************begin the inner loop******************************/
    int innercount = 0;
    double r = -1.0;

    double new_residual_cost;
    while (innercount < INNER_ITERATION_NUMBER)
    {
      ++innercount;
      // do make schur_matrix residual_c parameter_2_hessian init
      Schur_A.setZero();
      Schur_B.setZero();
      for (typename std::vector<BoundleAdjustment::Problem<
               N, N1, N2>::Residual_block *>::iterator it =
               residual_block.begin();
           it != residual_block.end(); ++it)
      {
        int id_a = (*it)->parameter_a;
        int id_b = (*it)->parameter_b;
        Schur_A.block<N1, N1>(id_a * N1, id_a * N1).noalias() +=
            (*it)->jacobi_parameter_1.transpose().lazyProduct(
                (*it)->jacobi_parameter_1);
        Schur_B.segment<N1>(id_a * N1).noalias() -=
            (*it)->jacobi_parameter_1.transpose().lazyProduct((*it)->residual);
        parameter_2_vector[id_b]->hessian.noalias() +=
            (*it)->jacobi_parameter_2.transpose().lazyProduct(
                (*it)->jacobi_parameter_2);
        parameter_2_vector[id_b]->residual.noalias() -=
            (*it)->jacobi_parameter_2.transpose().lazyProduct((*it)->residual);
      }

      Schur_A.diagonal().noalias() += 1 / Miu * Schur_A.diagonal();

      for (int i = 0; i < parameter_2_vector_length; ++i)
      {
        // update the parameter_2_hessian by Miu and compute the inverse

        parameter_2_vector[i]->hessian.diagonal().noalias() +=
            1 / Miu * parameter_2_vector[i]->hessian.diagonal();
        parameter_2_vector[i]->hessian_inverse =
            parameter_2_vector[i]->hessian.inverse();
      }

      schur_complement();

      for (int i = 0; i < residual_node_length; ++i)
      {
        int id = residual_block[i]->parameter_b;
        parameter_2_vector[id]->residual.noalias() -=
            residual_block[i]->hessian_W.transpose().lazyProduct(
                parameter_1_vector[residual_block[i]->parameter_a]->delta);
      }
      for (int i = 0; i < parameter_2_vector_length; ++i)
      {
        parameter_2_vector[i]->delta.noalias() =
            parameter_2_vector[i]->hessian_inverse.lazyProduct(
                parameter_2_vector[i]->residual);
      }
      model_cost = 0.0;

      for (typename std::vector<BoundleAdjustment::Problem<
               N, N1, N2>::Residual_block *>::iterator it =
               residual_block.begin();
           it != residual_block.end(); ++it)
      {
        Eigen::Matrix<double, N, 1> delta_parameter;
        int id_1 = (*it)->parameter_a;
        int id_2 = (*it)->parameter_b;
        delta_parameter = (*it)->jacobi_parameter_1.lazyProduct(
                              parameter_1_vector[id_1]->delta) +
                          (*it)->jacobi_parameter_2.lazyProduct(
                              parameter_2_vector[id_2]->delta);
        model_cost += (delta_parameter.transpose() *
                       (2 * (*it)->residual + delta_parameter))(0, 0);
      }
      model_cost = model_cost / 2;
      cout << "model cost:" << model_cost << endl;
      if (model_cost < 0)
      {
        for (int i = 0; i < parameter_1_vector_length; ++i)
        {
          parameter_1_vector[i]->delta.array() *=
              parameter_1_vector[i]->jacobi_scaling.array();
        }
        for (int i = 0; i < parameter_2_vector_length; ++i)
        {
          parameter_2_vector[i]->delta.array() *=
              parameter_2_vector[i]->jacobi_scaling.array();
        }
      }
      new_residual_cost = 0.0;
      for (typename std::vector<BoundleAdjustment::Problem<
               N, N1, N2>::Residual_block *>::iterator it =
               residual_block.begin();
           it != residual_block.end(); ++it)
      {
        int id_1 = (*it)->parameter_a;
        int id_2 = (*it)->parameter_b;
        parameter_1_vector[id_1]->candidate =
            parameter_1_vector[id_1]->params + parameter_1_vector[id_1]->delta;
        parameter_2_vector[id_2]->candidate =
            parameter_2_vector[id_2]->params + parameter_2_vector[id_2]->delta;
        (*it)->residual_node->computeResidual(
            &parameter_1_vector[id_1]->candidate,
            &parameter_2_vector[id_2]->candidate, &new_residual_cost);
      }
      new_residual_cost /= 2;
      double relative_decrease =
          (new_residual_cost - current_cost) / model_cost;
      double historical_relative_decrease =
          (new_residual_cost - reference_cost) /
          (accumulated_reference_model_cost_change + model_cost);
      r = relative_decrease < historical_relative_decrease
              ? historical_relative_decrease
              : relative_decrease;

      if (r > 0.001)
      {
        Miu = min(Miu / max(1.0 / 3.0, 1.0 - pow((2 * r - 1.0), 3)), 1e16);
        v0 = 2;
        break;
      }
      else
      {
        Miu = Miu / v0;
        v0 *= 2;
      }
    }
    /*****************end the inner loop********************************/
    if ((residual_cost - new_residual_cost) / residual_cost < MINDIF)
    {
      cout << "leave by MINDIF reached!" << endl;
      break;
    }
    double step = 0.0;
    if (!update_parameter(&step))
    {
      cout << "reach the parameter limit :" << PARAMETERMIN << endl;
      break;
    }
    for (typename std::vector<
             BoundleAdjustment::Problem<N, N1, N2>::Residual_block *>::iterator
             it = residual_block.begin();
         it != residual_block.end(); ++it)
    {
      (*it)->residual_node->computeJacobiandResidual(
          &parameter_1_vector[(*it)->parameter_a]->params,
          &parameter_2_vector[(*it)->parameter_b]->params,
          &(*it)->jacobi_parameter_1, &(*it)->jacobi_parameter_2,
          &(*it)->residual);
    }
    printf("%4s(%d)", std::to_string(outcount).c_str(), innercount);
    printf("%20s", std::to_string(new_residual_cost).c_str());
    printf("%20s", std::to_string(residual_cost).c_str());
    printf("%20s", std::to_string(step).c_str());
    printf("%20s", std::to_string(r).c_str());
    printf("%20s", std::to_string(totaltime).c_str());
    printf("\n");
    residual_cost = new_residual_cost;
  }
  post_process();
}
template <int N, int N1, int N2>
void Problem<N, N1, N2>::post_process()
{
  map<double *, int>::iterator iter;
  for (iter = parameter_1_map.begin(); iter != parameter_1_map.end(); ++iter)
  {
    Eigen::Map<Eigen::Matrix<double, N1, 1>>(
        iter->first, parameter_1_vector[iter->second]->params.rows(), 1) =
        parameter_1_vector[iter->second]->params;
  }
  for (iter = parameter_2_map.begin(); iter != parameter_2_map.end(); ++iter)
  {
    Eigen::Map<Eigen::Matrix<double, N2, 1>>(
        iter->first, parameter_2_vector[iter->second]->params.rows(), 1) =
        parameter_2_vector[iter->second]->params;
  }
}
}  // namespace BoundleAdjustment
