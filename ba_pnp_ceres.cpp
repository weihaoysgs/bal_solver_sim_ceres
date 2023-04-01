#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "common/DataType.hpp"
#include "common/CeresRotation.hpp"
#include "common/ReadDataset.hpp"

using namespace Eigen;
using namespace ceres;
using namespace std;

class ReprojectionError
{
 public:
  ReprojectionError(Eigen::Vector3d point_, Eigen::Vector2d observed_)
      : point(point_), observed(observed_)
  {
  }

  template <typename T>
  bool operator()(const T *const camera_r, const T *const camera_t,
                  T *residuals) const
  {
    T pt1[3];
    pt1[0] = T(point.x());
    pt1[1] = T(point.y());
    pt1[2] = T(point.z());

    T pt2[3];
    ceres::AngleAxisRotatePoint(camera_r, pt1, pt2);

    pt2[0] = pt2[0] + camera_t[0];
    pt2[1] = pt2[1] + camera_t[1];
    pt2[2] = pt2[2] + camera_t[2];

    const T xp = T(K[0] * (pt2[0] / pt2[2]) + K[2]);
    const T yp = T(K[1] * (pt2[1] / pt2[2]) + K[3]);

    const T u = T(observed.x());
    const T v = T(observed.y());

    residuals[0] = u - xp;
    residuals[1] = v - yp;

    return true;
  }

  static ceres::CostFunction *Create(Eigen::Vector3d points,
                                     Eigen::Vector2d observed)
  {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
        new ReprojectionError(points, observed)));
  }

 private:
  Eigen::Vector3d point;
  Eigen::Vector2d observed;
  // Camera intrinsics
  double K[4] = {520.9, 521.0, 325.1, 249.7};  // fx,fy,cx,cy
};

int main(int argc, char *argv[])
{

  std::vector<Eigen::Vector3d> p3d;
  std::vector<Eigen::Vector2d> p2d;

  const std::string pts_3d_file = "../data/p3d.txt";
  const std::string pts_2d_file = "../data/p2d.txt";
  ReadPoint2dDataset(pts_2d_file, pts_3d_file, p3d, p2d, false);

  ceres::Problem problem;
  ceres::LossFunction *lossfunction = NULL;
  double camera_rvec[3] = {0, 0, 0};
  double camera_t[3] = {0, 0, 0};

  for (uint i = 0; i < p3d.size(); i++)
  {
    Eigen::Vector3d p3dVec(p3d[i](0), p3d[i](1), p3d[i](2));
    Eigen::Vector2d p2dVec(p2d[i](0), p2d[i](1));
    ceres::CostFunction *costfunction =
        ReprojectionError::Create(p3dVec, p2dVec);
    problem.AddResidualBlock(costfunction, lossfunction, camera_rvec, camera_t);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 100;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  Eigen::Quaterniond q = AngleAxisRotation2Quaternion(camera_rvec);
  Eigen::Matrix3d rMatrix = q.toRotationMatrix();

  Eigen::Matrix4d T;
  T << rMatrix(0, 0), rMatrix(0, 1), rMatrix(0, 2), camera_t[0], rMatrix(1, 0),
      rMatrix(1, 1), rMatrix(1, 2), camera_t[1], rMatrix(2, 0), rMatrix(2, 1),
      rMatrix(2, 2), camera_t[2], 0, 0, 0, 1;
  std::cout << "T = \n" << T << std::endl;
  return 0;
}