#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>

#include "BoundleAdjustmentByNode_accelebrate.h"
#include "common/CeresRotation.hpp"
#include "common/ReadDataset.hpp"

using namespace BoundleAdjustment;

class CostFunctionBA
{
 public:
  Eigen::Vector3d pts_3d_w_;
  Eigen::Vector2d pts_2d_observed_;
  double K[4] = {520.9, 521.0, 325.1, 249.7};  /// fx,fy,cx,cy
  CostFunctionBA(Eigen::Vector3d pts_3d, Eigen::Vector2d pts_2d)
      : pts_2d_observed_(pts_2d), pts_3d_w_(pts_3d)
  {
  }
  template <class T>
  void Evaluate(const T *camera_r, const T *camera_t, T *residual)
  {
    T pts_w[3];
    pts_w[0] = T(pts_3d_w_.x());
    pts_w[1] = T(pts_3d_w_.y());
    pts_w[2] = T(pts_3d_w_.z());

    T pts_camera[3];
		AngleAxisRotatePoint(camera_r, pts_w, pts_camera);
    // ceres::AngleAxisRotatePoint(camera_r, pts_w, pts_camera);
    pts_camera[0] = pts_camera[0] + camera_t[0];
    pts_camera[1] = pts_camera[1] + camera_t[1];
    pts_camera[2] = pts_camera[2] + camera_t[2];

    const T u_predict = T((pts_camera[0] / pts_camera[2]) * K[0]) + T(K[2]);
    const T v_predict = T((pts_camera[1] / pts_camera[2]) * K[1]) + T(K[3]);

    const T observe_u = T(pts_2d_observed_.x());
    const T observe_v = T(pts_2d_observed_.y());

    residual[0] = u_predict - observe_u;
    residual[1] = v_predict - observe_v;
  }
};

int main(int argc, char const *argv[])
{
  std::vector<Eigen::Vector3d> pts_3d;
  std::vector<Eigen::Vector2d> pts_2d;

  const std::string pts_3d_file = "../data/p3d.txt";
  const std::string pts_2d_file = "../data/p2d.txt";
  ReadPoint2dDataset(pts_2d_file, pts_3d_file, pts_3d, pts_2d, false);
  double camera_rvec[3] = {0, 0, 0};
  double camera_t[3] = {0, 0, 0};

  BoundleAdjustment::Problem<2, 3, 3> problem;
  for (size_t i{0}; i < pts_2d.size(); i++)
  {
    Residual_node<CostFunctionBA, 2, 3, 3> *residual_node =
        new Residual_node<CostFunctionBA, 2, 3, 3>(
            new CostFunctionBA(pts_3d[i], pts_2d[i]));
    problem.addParameterBlock(camera_rvec, camera_t, residual_node);
  }
  problem.solve();
  std::cout << "////////////////////////////////\n";
  for (auto &t : camera_t)
  {
    std::cout << t << "\t";
  }
	std::cout << std::endl;
  Eigen::Quaterniond q = AngleAxisRotation2Quaternion(camera_rvec);
  std::cout << "rotation:\n" << q.toRotationMatrix() << std::endl;
  return 0;
}
